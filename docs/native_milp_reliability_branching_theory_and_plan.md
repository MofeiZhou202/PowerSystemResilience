# Native MILP reliability branching: theory, contracts, and implementation plan

Status: implementation, Phase F evaluation, the D16 correctness re-audit, and
the P13 root-probe transaction are recorded through 2026-08-02.
This document records the design, implementation contracts, and experimental
decisions. It does not claim that Native is already faster than HiGHS and does
not authorize instance-specific tuning.

## 1. Scope and performance objective

This design covers one bounded subsystem: reliability branching, reuse of
strong-branch LP states, child-direction scheduling, and the telemetry needed
to attribute their effects. It does not mix in cut-policy, presolve, node-LP
kernel, or instance-specific parameter changes.

For a fixed model, seed, thread count, and stopping rule, decompose MILP time as

\[
T = T_{root} + \sum_{v\in V}
    (T_{prop,v}+T_{LP,v}+T_{cut,v}+T_{queue,v}) + T_{heur}.
\]

Branching can improve total time by reducing the explored node set \(V\), by
finding an incumbent early enough to prune queued nodes, and by reusing probe
LP work. It can also lose time when probes do not affect the selected branch or
when a selected probe is solved again. Therefore node count alone is not the
objective. The primary performance criterion is paired wall time; node count,
LP solves, LP iterations, time to first incumbent, gap integral, and final gap
are explanatory metrics.

An earlier fixed four-case branching diagnostic showed a useful but incomplete
mechanism signal:
the node geometric mean fell from 51.392 to 41.478, while `enlight_hard` used
58 strong LP solves but recorded zero exact hits, zero warm hits, and zero
strong-regret samples. This proves that branching behavior changed; it does not
prove that the probe investment was used efficiently, that wall time improved,
or that Native beats HiGHS. Section 19 records the newer post-correctness-fix
solver comparison.

## 2. Branching model and exact directional gain

Assume the active relaxation is represented in minimization form. Let \(D\) be
the current node domain, \(z(D)\) its valid LP lower bound, and let an integer
candidate satisfy

\[
x_j^*=k+f_j,\qquad k=\lfloor x_j^*\rfloor,\qquad 0<f_j<1.
\]

The branching disjunction creates

\[
D_j^- = D\cap\{x_j\le k\},\qquad
D_j^+ = D\cap\{x_j\ge k+1\}.
\]

If the two child relaxations are solved to valid optima, their directional
gains are

\[
g_j^- = \max(0,z(D_j^-)-z(D)),\qquad
g_j^+ = \max(0,z(D_j^+)-z(D)).
\]

The max with zero protects the branching estimator from harmless numerical
nonmonotonicity. It is not a substitute for the bound audit: a child bound may
be published only after the LP status and active-domain contract are valid.

A useful branch should improve both sides of the disjunction. The existing
product score expresses that preference:

\[
S_j = \max(g_j^-,\epsilon)\max(g_j^+,\epsilon).
\]

This phase keeps the product rule unchanged so the effect of exact local
information can be isolated. Branch priority, the existing inference/conflict/
cutoff history multiplier, and an installed dynamic prior keep their current
combination semantics. No new score weight is introduced.

## 3. Pseudocost normalization

Strong branching is too expensive at every node, so future gains are estimated
from directional unit costs:

\[
p_j^- = \frac{g_j^-}{f_j},\qquad
p_j^+ = \frac{g_j^+}{1-f_j}.
\]

At another node with fractional part \(f\), the predictions are

\[
\hat g_j^- = f\hat p_j^-,\qquad
\hat g_j^+ = (1-f)\hat p_j^+.
\]

The displacement must be captured before child propagation. Propagation may
tighten the child and improve its final bound, but it must not redefine the
distance of the original branch observation. Storing an unnormalised gain and
multiplying by distance again would make the estimate quadratic in
fractionality and make samples from different nodes incomparable.

One LP evaluation contributes at most one directional pseudocost sample. If a
strong result is consumed as the actual child result, child processing must not
record the same objective gain a second time. Inference and conflict events
created later by child propagation remain distinct observations.

## 4. Why the current update-then-select rule loses information

Let \(\hat p_n\) be the mean of \(n\) historical unit-cost observations. A
current strong probe at distance \(d\) observes exact gain \(g\). Updating the
global mean first gives

\[
\hat p_{n+1}=\frac{n\hat p_n+g/d}{n+1}.
\]

If the current candidate is then rescored from that mean, the supposedly strong
observation becomes

\[
d\hat p_{n+1}
=\frac{n}{n+1}d\hat p_n+\frac{1}{n+1}g.
\]

Thus the current exact observation has weight only \(1/(n+1)\), while stale
history has weight \(n/(n+1)\). This is the opposite of the intended
information ordering and explains how a probed provisional winner can be
replaced by an unprobed variable after the probes have already been paid for.

The statistical statement is also direct. Let \(H\) be historical information
and \(Y\) the current-node strong observation. For squared prediction loss,

\[
E[(g-E[g\mid H,Y])^2]\le E[(g-E[g\mid H])^2].
\]

When \(Y\) is a completed current-node solve, its directional gain is the local
observation. Averaging it back into \(H\) before making the same-node decision
throws away conditioning information.

## 5. Node-local exact-score overlay

Each candidate direction has a logical evidence state:

1. `Predicted`: no usable current-node result; use pseudocost prediction.
2. `Measured`: a finite, audited current-node LP objective is usable for branch
   ranking, but is not necessarily a proof that may be published as a bound.
3. `ExactOptimal`: the current child relaxation was solved under a valid
   optimality contract.
4. `ProvenCutoff`: infeasibility or objective cutoff has a valid certificate.
5. `UnknownFailure`: timeout, iteration limit without a valid dual bound,
   numerical failure, unclassified status, or unusable unbounded status.

The local directional score is

\[
\tilde g_{j,d}=
\begin{cases}
g_{j,d}^{local},&d\text{ is Measured or ExactOptimal},\\
\delta_{j,d}\hat p_{j,d},&d\text{ is Predicted or UnknownFailure}.
\end{cases}
\]

`UnknownFailure` deliberately falls back to the pre-probe prediction. It does
not create a pseudocost sample. `ProvenCutoff` is a discrete proof state, not a
large floating-point gain.

The final comparison covers every eligible fractional candidate:

- a probed direction uses its current-node observation;
- a reliable, unprobed direction uses its pseudocost prediction;
- an unreliable, unprobed direction uses the existing fallback estimate;
- branch priority and dynamic priors are applied by the same final scorer used
  before probing.

This is an overlay, not a separate branching policy. Current-node observations
disappear when the node is left; their normalized samples remain in the global
pseudocost table for future nodes.

## 6. Proof states must not be encoded as `1e6`

Using `1e6` for a failed probe has three defects:

1. a numerical failure becomes indistinguishable from proven infeasibility;
2. the decision changes under objective rescaling;
3. the artificial value contaminates the mean and variance at future nodes.

The required transition table is:

| LP outcome | Local branch score | Pseudocost update | Child action |
|---|---|---|---|
| audited optimal success | observed gain | one unit-gain sample | exact reuse if signatures match |
| finite heuristic success | measured gain only | update only if the solver contract permits | warm or exact according to proof status |
| Farkas-certified infeasible | `ProvenCutoff` | cutoff/conflict counters, no fake gain | prune direction or infer opposite bound |
| certified objective cutoff | `ProvenCutoff` | cutoff counter, no fake gain | prune direction |
| timeout/iteration limit | prediction | no gain/cutoff sample | solve normally if selected |
| numerical/unknown failure | prediction | no gain/cutoff sample | solve normally with fallback if selected |

If both directions are proven cut off, the current node is closed. If exactly
one direction is proven cut off, the opposite branch bound is a node-local
domain implication. Applying it is stronger than constructing a dead child,
but it must be transactional: update the node domain, rerun domain closure and
the LP, invalidate the old candidate set and all node-local probe scores, then
restart selection. A locally scoped proof must never be published globally.

## 7. Complete reliability-branching algorithm

The algorithm follows a candidate-driven loop rather than a one-shot batch
whose results are immediately compressed into global averages.

```text
INPUT: node D, relaxation (x*, z), fractional candidates C,
       pseudocost table P, existing probe budget B

1. For every j in C:
     initialise down/up local evidence as Predicted
     compute predicted directional gains from P

2. Repeat:
     a. Select the current best candidate over all C using the final scorer:
          priority -> local-overlay product score -> existing prior/tie rules
     b. Determine which directions of that candidate are neither locally
        observed nor pseudocost-reliable.
     c. If no such direction exists or B is exhausted, stop.
     d. Probe the required direction(s) from the immutable parent snapshot.
     e. Classify each result with the transition table in Section 6.
     f. Store the node-local result before updating global pseudocost history.
     g. Update global statistics exactly once for every valid observation.
     h. Audit and adopt any integral probe solution through the normal
        incumbent path.
     i. If a proof implies a node-domain reduction, apply it transactionally,
        invalidate this candidate state, re-solve the node, and restart at 1.

3. Recompute the final winner over all C using node-local overlays.

4. Retain the selected candidate's down/up probe states.

5. Process children in the preferred direction order. Consume an exact probe
   only when the child signature matches; otherwise use it only as a warm state.

6. Queue surviving children with the existing node-selection policy.
```

The loop has an important economic property. After probing changes the winner,
the new winner is itself checked for reliability before selection completes.
This is the behavior visible in the HiGHS candidate-state arrays and selection
loop, and it avoids spending the entire budget on candidates that are no longer
competitive.

The existing candidate and LP budgets remain unchanged during the first
implementation. Budget redesign is a separate experiment and must not be mixed
with the estimator correction.

## 8. Strong-result reuse contract

Branch ranking and child-bound publication require different contracts.

A successful probe may be used as a local ranking observation even if later
propagation tightens the selected child. It may be consumed as the actual child
LP result only if all of the following match:

- variable and direction;
- complete lower and upper bounds after domain closure;
- active local-cut row structure and ordering;
- objective and model epoch relevant to the solve;
- solver status required for the intended use;
- no temporary basis object retains a pointer to a destroyed matrix.

For probe domain \(D_p\) and closed child domain \(D_c\):

- if \(D_p=D_c\) and the row signature agrees, objective/status/basis may be
  consumed under their proof contract;
- if \(D_c\subset D_p\), the old objective is not the child bound, but the
  clamped primal and independently owned basis may warm the re-solve;
- if the row structure differs incompatibly, only explicitly structure-safe
  state may be retained;
- native sparse factorization objects tied to a temporary matrix are detached.

Probe execution starts from an immutable parent snapshot. Rejection or failure
restores bounds, rows, basis metadata, and factorization state. A sibling must
never observe mutations made by the other probe.

The performance value of reuse is

\[
T_{overhead}\approx
T_{unselected\ probes}+T_{domain\ mismatch\ resolves}
-T_{selected\ duplicate\ LPs\ avoided}.
\]

When the selected pair is exact and reusable, its two strong LPs replace the
two normal child LPs instead of adding two extra LPs. This is why selected-pair
coverage and exact-hit telemetry are central performance measures.

## 9. Child-direction scheduling

Variable selection controls tree shape; direction selection controls when a
useful subtree is explored. The current sequential path always processes down
before up, even when strong branching already measured both directions.

Direction scheduling is applied only after the exact-overlay change has been
validated:

- a probe-discovered integral solution is audited and adopted immediately;
- without an incumbent, process the direction with the smaller observed or
  predicted child lower bound first, matching primal-oriented diving;
- with an incumbent, a proof-bearing cutoff is applied before solving the
  sibling; otherwise retain the existing search-mode preference;
- an explicit callback direction remains an advisory input under its existing
  contract;
- after both children are processed, the same preferred direction is the DFS
  child unless proof-tail queue mode overrides it.

Processing the preferred child first matters even when both children are
eventually evaluated: a newly audited incumbent can tighten the objective limit
used for the second child and can prune queued nodes before more work is spent.

## 10. Pseudocost reliability and shrinkage

The current 95% lower confidence bound is not changed in the exact-overlay
experiment. It has a known discontinuity: one observation uses the raw mean,
while two noisy observations can suddenly collapse the lower bound toward
epsilon. That can create ranking oscillation.

After overlay attribution is complete, unreliable estimates should be studied
with global directional shrinkage:

\[
\tilde p_{j,d}=w(n_{j,d})\bar p_{j,d}
 +(1-w(n_{j,d}))\bar p_{global,d},
\qquad w(0)=0,\quad w(n)\uparrow1.
\]

This is a cohort-wide estimator, not an instance parameter. It is accepted
only after calibration data show lower directional prediction error. It must
not be introduced in the same benchmark comparison as the local overlay.

## 11. Node estimate is a separate estimator

The queue estimate currently adds

\[
z(D)+\sum_{j\ fractional}\min(\hat g_j^-,\hat g_j^+).
\]

Correlated or degenerate variables can make that sum count the same future
bound improvement repeatedly. This can damage final gap by misordering the
queue, but changing it at the same time as branching would destroy attribution.

First record predicted estimate lift versus realised child-bound lift and rank
correlation. Only if the estimator is systematically overconfident should a
replacement such as a maximum or calibrated aggregation be evaluated. No
top-k constant or case-dependent cap is introduced without this evidence.

## 12. Telemetry and regret definitions

The minimum additional counters are:

- `strong_complete_probe_pairs`;
- `strong_selected_exact`;
- `strong_selected_unprobed`;
- `strong_winner_changed_by_exact`;
- `strong_probe_unknown_failures`;
- `strong_probe_lp_iterations` and `strong_probe_time_ms`;
- exact/warm cache hits and duplicate LPs avoided, already present;
- preferred direction, first processed direction, and incumbent found there;
- first incumbent source, depth, node count, LP count, and wall time;
- predicted node-estimate lift and realised child-bound lift.

Pseudocost regret and observed strong regret answer different questions:

\[
R_{pc}=\max_{i\in C_{prio}}S_i^{pc}-S_j^{pc},
\]

\[
R_{sb}=\max_{i\in C_{complete}}S_i^{local}-S_j^{local}.
\]

`R_sb` exists only if the selected variable has a complete usable pair. Zero
samples means missing coverage, not zero regret. Reports must publish both the
sample count and the regret aggregate. The more direct probe-efficiency metric
is

\[
\rho_{reuse}=\frac{\text{selected exact directions consumed}}
                   {\text{strong direction LPs solved}}.
\]

## 13. Mathematical contract tests

The implementation is incomplete until the following deterministic tests pass:

1. Unit gain is `gain / original branch distance` for root probes, tree probes,
   and solved children.
2. A current-node exact pair overrides arbitrarily large historical sample
   counts in current-node selection.
3. One exact direction combines with the other predicted direction.
4. An unprobed reliable candidate can still beat a probed candidate.
5. Priority and dynamic-prior behavior are unchanged by the overlay.
6. Timeout, numerical failure, and unknown status add no gain or cutoff sample.
7. Certified infeasibility changes only proof/cutoff/conflict state and never
   injects a scale-dependent fake gain.
8. One physical LP result contributes at most one pseudocost gain sample.
9. A complete selected probe pair produces exact cache hits when domain and row
   signatures match.
10. A stricter propagated child domain produces a warm hit, not an exact bound.
11. A local-cut mismatch prevents exact reuse.
12. Failed or rejected probes restore model, bounds, basis, and factorization;
    the sibling solve matches a clean-parent solve.
13. Probe incumbents pass bound, row, integrality, and recomputed-objective
    audits before adoption.
14. Serial and parallel selectors agree on a synthetic candidate table with
    the same priorities, pseudocosts, and local observations.

## 14. Code implementation map

The implementation should replace duplicated logic rather than add a second
branching subsystem.

### Phase A: evidence and failure contracts

- In `bc_branching.cpp`, expose one directional/product score helper used by
  dense, compact, sequential, and parallel paths.
- Replace every unconditional `normalized_pseudocost_gain(1e6, ...)` failure
  update with the transition table in Section 6.
- Add the failure and one-solve/one-sample contract tests first.

### Phase B: node-local overlay and candidate loop

- In `branch_and_cut.cpp`, use the existing `probe_results` storage. Add small
  lookup helpers for directional local gain/status; do not add a persistent
  candidate class.
- Replace the update-then-global-reselect block with the loop in Section 7.
- Replace the current strong-regret scan with the same local score helper, so
  decision and telemetry cannot disagree.
- Preserve final full-candidate comparison, priorities, and dynamic priors.

### Phase C: selected-state consumption and sample deduplication

- Retain both selected directional states as today.
- Centralise the exact-domain/row-signature check used by child processing.
- Mark the source observation consumed so IPM and simplex paths cannot update
  the same gain twice.
- Keep native temporary matrix-bound factors detached and vendored-HiGHS basis
  ownership independent.

### Phase D: parallel semantic parity

- Change `choose_branch_var_reliability_impl` to use the same local overlay.
- Do not let the parallel path return to global pseudocost selection after
  discarding current probe values.
- Full factorization reuse remains worker-local; score semantics must still be
  identical when the inputs are identical.

### Phase E: direction scheduling

- Replace the fixed down/up execution order with one shared child-processing
  call driven by a two-entry direction order.
- Do not duplicate the existing large child-processing blocks.
- Measure first-incumbent and second-child cutoff effects separately.

### Phase F: later estimator work

- Evaluate pseudocost shrinkage only after Phases A-E are stable.
- Instrument node-estimate calibration before changing queue estimates.
- Do not tune probe counts, reliability thresholds, score weights, or
  model-size branches against individual MIPLIB instances.

## 15. Validation and acceptance gates

Every phase uses the same immutable benchmark manifest, seeds, time limits,
thread counts, solver builds, and mathematical audits. Instances are not added
or removed after seeing performance results.

The gates are cumulative:

1. **Correctness:** all existing MILP/B&C tests plus Section 13 pass; every
   returned incumbent and published bound passes mathematical audit.
2. **Mechanism:** strong-regret coverage becomes nonzero on cases that probe;
   selected exact/warm hits and avoided duplicate LPs agree arithmetically.
3. **Efficiency:** paired strong LP iterations and wall time explain any LP
   solve reduction; probe overhead is not hidden by node-count reporting.
4. **Search quality:** node shifted/geometric mean, first-incumbent time, gap
   integral, and final gap are reported together.
5. **End-to-end:** only paired wall-time confidence intervals over the fixed
   MIPLIB cohort support a faster-than-HiGHS claim.

The first implementation comparison changes only Phases A and B. Phase C may
then establish whether the corrected winner produces actual reuse. Direction
scheduling, shrinkage, and node-estimate changes each require their own paired
before/after report. This staged order is for causal attribution, not repeated
parameter tuning.

## 16. Current code locations and external reference behavior

- Native product scoring: `src/engine/solver/native/milp/bc/legacy/bc_branching.cpp`.
- Sequential probes, selection, retained states, and children:
  `src/engine/solver/native/milp/bc/legacy/branch_and_cut.cpp`.
- Parallel reliability path:
  `src/engine/solver/native/milp/bc/legacy/bc_parallel.cpp` and
  `choose_branch_var_reliability_impl`.
- Pseudocost state: `include/mipsolvers/engine/detail/bc_types.hpp`.
- Statistics export: `include/mipsolvers/engine/bc/stats.hpp` and
  `src/python/mipsolvers_py.cpp`.
- HiGHS keeps current-node `upscore`, `downscore`, reliability flags, and child
  bounds in `highs/mip/HighsSearch.cpp` and selects repeatedly until the winner
  is reliable or its strong-iteration budget is exhausted.
- SCIP reuses a strong score recorded at the current node in
  `scip/scip/branch_relpscost.c` instead of reconstructing that score only from
  global pseudocost averages; proof-bearing cutoff directions can produce
  local bound changes.

Primary theory references are Achterberg, Koch, and Martin, *Branching Rules
Revisited* (2005); Achterberg, *Constraint Integer Programming* (2007); and Le
Bodic and Nemhauser, *An Abstract Model for Branching and its Application to
Mixed Integer Programming* (2017). The vendored HiGHS and SCIP sources above
are the implementation references for the node-local state machine and proof
validity behavior.

## 17. Implementation progress, 2026-08-01

Phases A-E are complete:

- Phase A now has shared directional/product scoring, explicit evidence states,
  and a single probe-to-pseudocost transition helper. Unknown failures add no
  gain or cutoff sample, and proven cutoffs no longer inject `1e6` gains.
- Phase B now uses current-node overlays in the sequential simplex, IPM, and
  shared reliability selectors. Candidate probing reselects after each current
  winner pair while preserving static priority and dynamic-prior semantics.
- Strong regret uses the same overlay scorer as selection. New mechanism
  telemetry records complete pairs, selected exact/unprobed outcomes, winner
  changes, unknown failures, probe iterations, and probe wall time.
- The sequential current-node prober and the shared reliability helper used by
  parallel search now materialize one active standard form per probing batch.
  Each directional probe applies an exact `StandardFormBoundTransaction`, solves
  against the active form, and rolls back only the touched bound, RHS, and
  objective scalars. Per-direction standard-form copies have been removed.
  Telemetry separates base materializations, transactions, snapshotted values,
  rollbacks, failures, persistent resolves, and cold solves. This establishes a
  structural copy reduction; it does not establish an end-to-end speedup.
- Phase C centralizes exact/warm/incompatible probe reuse. Exact replacement
  requires the same closed domain, ordered local-row signature, model and
  objective epochs, and proof-bearing simplex evidence. Strictly tighter child
  domains may use compatible warm state; consumed or stale probes cannot be
  reused. The one-physical-result/one-gain-sample guard is installed for exact
  probe consumption, while later propagation observations remain independent.
- Phase D routes shallow Prover reliability probing and deep Prover/Diver
  selection through the shared product score. Parallel explorers receive an
  immutable static-priority snapshot, original-column mapping, and optional
  dynamic prior; callback contexts expose original-space candidate and LP
  columns. Candidate-aligned pseudocost snapshots avoid a model-sized copy in
  the fast parallel path.
- Phase E uses one shared direction-order rule in sequential and parallel
  child evaluation. Before an incumbent it processes the smaller locally
  observed or predicted child-bound lift first; with an incumbent a unique
  proof-bearing cutoff takes precedence and the existing down-first preference
  is otherwise retained. Both child evaluators are driven through one
  direction-parameterized call, and the chosen direction remains the DFS child
  unless proof-tail queue mode overrides DFS. Telemetry separates preferred
  and first directions, first-child incumbent updates, and first/second-child
  cutoffs.
- Phase F calibration instrumentation is installed without changing either
  estimator. Sequential and parallel expansions record queued node-estimate
  lift versus the best realised child-bound lift, including moments needed for
  offline bias, RMSE, and Pearson-correlation analysis. History-only
  directional predictions record realised-gain error and two-child rank
  concordance; directions already measured by a current-node probe are
  excluded from directional prediction-error samples to prevent leakage.
- Deterministic tests cover normalization, exact-over-history selection,
  mixed measured/predicted directions, reliable unprobed winners, priority and
  dynamic-prior preservation, failure transitions, sample deduplication,
  exact/warm reuse signatures, and serial/parallel selector parity.

Phase F behavioral evaluation is complete. A parameter-free `Maximum`
node-estimate aggregation was implemented as an explicit experiment while the
production default remains `Sum`. The fixed-cohort repetitions did not show a
meaningful wall-time improvement, so `Maximum` was not promoted. Directional
calibration did not support one cohort-wide pseudocost shrinkage rule, so no
shrinkage weight was introduced. Probe budgets, reliability thresholds, and
branch-score weights remain unchanged.

## 18. Phase F numerical experiment, 2026-08-01

### 18.1 Protocol and artifacts

The fixed diagnostic cohort was `50v-10`, `enlight_hard`,
`neos-3083819-nubu`, and `wachplan`. Both node-estimate modes used the same
MIPLIB data, Native solver build, one thread, 10 second limit, relative gap
`1e-4`, and 50,000-node limit. The runner requested CLI seed 0. Each final
comparison used three repetitions. The reports are:

- `reports/miplib2017_reliability_phasef_sum_repeat3_2026-08-01.{json,csv}`;
- `reports/miplib2017_reliability_phasef_maximum_repeat3_2026-08-01.{json,csv}`;
- the corresponding single-run baseline and maximum reports;
- `reports/miplib2017_reliability_phasef_sample12_{sum,maximum}_2026-08-01.{json,csv}`.

The repeated fixed-cohort command was:

```sh
./tests/miplib2017_benchmark \
  --data-dir tests/data/miplib2017/benchmark \
  --solu tests/data/miplib2017/miplib2017-v36.solu \
  --case 50v-10,enlight_hard,neos-3083819-nubu,wachplan \
  --solvers native-highs-lp --repeat 3 --time-limit 10 \
  --gap 1e-4 --max-nodes 50000 --seed 0 \
  --native-node-estimate sum
```

The `Maximum` run changed only the last argument to `maximum` and used a
different output path. The deterministic 12-case check used `--sample 12`, a
3 second limit, and one repetition.

#### Seed-provenance correction (2026-08-02)

The command and report JSON correctly record the requested CLI seed, but the
pre-D10 benchmark did not assign `cfg.seed` to Native `BCOptions`. These runs
therefore used the Native default `0x9E3779B97F4A7C15` for their heuristic RNG,
not Native seed 0. The two modes still shared the same default configuration,
so the results remain a diagnostic `Sum`/`Maximum` comparison, but they are not
evidence for the previously claimed effective-zero-seed or fully replayable
protocol. D10 subsequently wired the CLI seed into Native and made every seed,
including zero, deterministic throughout the work-stealing path. Only reports
created after D10 satisfy that corrected seed-provenance contract.

### 18.2 Fixed-cohort results

| Instance | Sum mean ms | Maximum mean ms | Sum nodes | Maximum nodes |
|---|---:|---:|---:|---:|
| `50v-10` | 10014.9 | 10016.6 | 419, 411, 407 | 410, 406, 407 |
| `enlight_hard` | 547.7 | 546.3 | 120, 120, 120 | 120, 120, 120 |
| `neos-3083819-nubu` | 1997.4 | 1997.1 | 1, 1, 1 | 1, 1, 1 |
| `wachplan` | 9037.9 | 9080.7 | 72, 72, 72 | 72, 72, 72 |

The PAR-10 shifted mean was 40.9206 seconds for `Sum` and 40.9195 seconds for
`Maximum`. This 0.0011 second difference is not a meaningful performance
signal. `Maximum` reduced the first three `50v-10` node counts by 14 in total,
but did not improve the timeout-limited wall time; the other tree shapes were
identical.

Calibration explains why `Maximum` was worth testing but not promoting. On a
representative `Sum` repetition, predicted node-estimate lift versus realised
best-child lift was approximately 53,394 versus 157 for `50v-10`, 486 versus
43 for `enlight_hard`, and 2,476 versus 0 for `wachplan`. `Maximum` greatly
reduced the prediction scale, but this did not translate into a stable
end-to-end improvement. `neos-3083819-nubu` had no non-root calibration
samples.

Historical directional ordering was already informative: representative
rank concordance was 403/419 for `50v-10`, 31/41 for `enlight_hard`, and 64/64
for `wachplan`. Magnitude bias was not uniform: `50v-10` overpredicted in
aggregate, `enlight_hard` underpredicted, and `wachplan` predicted positive
gain where the realised aggregate was zero. Consequently, the Section 10
acceptance gate for one global directional shrinkage estimator was not met.

### 18.3 Stratified-sample limitation

The deterministic 12-case comparison produced zero audited incumbents and a
30 second PAR-10 score for both modes. Only `enlight_hard` entered the tree,
with 120 nodes and 819 LP solves in both modes. Most other cases failed in the
root relaxation and three exceeded the nominal 3 second limit while inside an
LP solve. This sample is retained as a robustness artifact, but it cannot
discriminate queue-estimate quality.

### 18.4 Decision

All unconditional work in Phases A-F is complete. `NodeEstimateAggregation`
exposes `Sum` and `Maximum`, the benchmark runner records the selected mode,
and `Sum` remains the behavior-preserving default. Pseudocost shrinkage is a
conditional proposal whose evidence gate failed, so implementing an arbitrary
weight would violate the plan. No instance-specific threshold, hidden model
branch, probe-count change, or reliability tuning was added.

### 18.5 Final validation

The final implementation was rebuilt through the `test_milp_solver`,
`test_branch_and_cut`, `test_l2o_trace`, and `miplib2017_benchmark` targets.
The focused regression results were 192 assertions in 27 MILP cases, 15
assertions in 4 branch-and-cut cases, and 99 assertions in 11 L2O cases. All
passed, and `git diff --check` reported no whitespace errors.

## 19. D16 correctness re-audit and frozen comparison, 2026-08-02

### 19.1 Cross-node reason-lifetime defect

SCIP's objective-37 solution for `enlight_hard` was used as an independent
feasible witness. Before the repair, Native tightened column 13 from `[0,1]` to
`[0,0]` at depth 4 under that witness. Source-tagged propagation replay located
the violating artifact in `conflict_pool`. The pool contained globally visible
unary claims including `x101 >= 2 infeasible`, `x105 <= 1 infeasible`, and
`x167 <= 0 infeasible`, all contradicted by the feasible witness.

The root cause was a lifetime mismatch. Row propagation computed a derived bound
and its `DomainReasonBound`, but the normal configuration did not persist that
reason with the child node. If a bound $b$ is valid only under antecedents $R$,
the globally consumable implication is

\[
R \Longrightarrow b,
\]

not $b$ in isolation. Resolving a later conflict with a strict subset
$R'\subset R$ is unsound unless $R'\Longrightarrow b$ is independently
proved. The child analyzer had only the incomplete local frontier and could
therefore turn a local conflict into an invalid global no-good.

Node-domain closure now stores row and proof propagation reasons for the same
lifetime as the bounds they justify. The optional diagnostic interface records
`DomainPropagationSource`, events, and failures only when an audit caller
requests them; the default production path does not allocate an event vector.
The row-round contract was also corrected: `max_rounds=0` performs no linear-row
round while retaining the separately budgeted conflict, clique, and implication
closure.

### 19.2 Failure evidence and post-repair witness checks

The pre-repair 10-round run exhausted the search queue after 198 nodes. Because
that run had excluded a known feasible objective-37 witness, the exhaustion is
failure evidence, not a performance result. After the reason-lifetime repair:

- three runs with the default three row rounds reached the time limit at
  530, 539, and 537 nodes, with no invalid-conflict publication and no witness
  violation;
- a 10-round stress run reached 868 nodes and the time limit, rather than the
  false 198-node exhaustion;
- the focused parent/child regression requires conflict analysis to resolve an
  inherited propagation reason back to the actual branch literal, without
  excluding an independent feasible witness;
- the zero-round regression requires the linear-row propagation budget to be
  honored exactly.

These checks close D16's specific proof-lifetime defect. Native still found no
incumbent on `enlight_hard` in these witness runs, so the result is neither a
general correctness proof nor evidence of improved efficiency.

### 19.3 Frozen four-instance solver comparison

The post-D16 report is
`reports/miplib2017_audit_d16_reason_persistence_2026-08-02.{json,csv}`. Its
immutable manifest records four instances, three repeats, one thread, seed zero,
a 3-second backend limit, 2-second watchdog grace, relative gap `1e-4`, and
Native reliability settings `probe_max=3`, directional reliability 2, and three
row-propagation rounds. The current runner spelling is `--case`, not the removed
`--instances` option.

| Solver | Feasible | Proven | Mean fixed-horizon PDI | Shifted PAR-10 |
|---|---:|---:|---:|---:|
| Native B&C + HiGHS LP | 6/12 | 0/12 | 1.984450 s | 30.000 s |
| HiGHS MIP 1.14.0 | 9/12 | 0/12 | 1.660431 s | 30.000 s |
| SCIP MIP 9.0.0 | 9/12 | 3/12 | 1.205239 s | 12.144 s |

The paired Native/HiGHS PDI ratio is 1.11159 with an instance-cluster bootstrap
95% interval `[0.86347, 1.65066]`. The Native/SCIP PDI ratio is 6.30388 with
interval `[0.92059, 255.46667]`; the Native/SCIP PAR-10 ratio is 11.22897. All
36 event streams are available, all 95 primal events carry original-space
vectors that pass the audit, and no call hit the hard watchdog.

The manifest payload SHA-256 is
`12dd95ba6a5875352e247d0f95b4cbe1e9695f0d387d9af6230a8447582ea79f`;
the final JSON SHA-256 is
`b829e6dc584c73e587e5bebcd10f4c9fe81ded32ba5bb3250a0b0e664630e2ff`.

This cohort provides no evidence that Native's overall efficiency improved.
The Native/HiGHS interval crosses one, Native proves no case, and SCIP is best on
the reported proof count, mean PDI, and PAR-10. Four instances are also too few
for a general ranking. P11 therefore remains `PARTIAL`, E08 remains `OPEN`, and
the broader structural items E01, E02, and P21 remain unresolved.

## 20. P13 root domain and LP probing transactions, 2026-08-02

### 20.1 Scope correction

The audit finding at
`docs/native_milp_math_model_and_audit.tex:1546` is primarily about Phase 1
root domain probing: after down and up propagation it repeated a down world and
performed an unconditional `O(n)` implication-extraction scan. The earlier
implementation removed the third world but left two complete `lb/ub` copies
and two full-column scans per binary. Treating the separate Phase 2 root LP
transaction as complete P13 closure was incorrect.

P13 is now evaluated over both paths:

- Phase 1 root domain probing owns sparse propagation, extraction, and rollback;
- Phase 2 root LP probing owns bound/SF transactions and LP-state isolation.

### 20.2 Root domain probing contract

`BCDomainProbeWorkspace` is a separately testable state owner. It initializes
one `BCDomain`, row activities, row-major views, changed-column stamps, and
literal stamps. Every direction executes:

1. save the trail and pending-row state;
2. fix one binary literal;
3. propagate only queued rows, then traverse clique neighbours of newly fixed
   literals;
4. collect unique columns from trail entries;
5. return sparse old/new bound deltas;
6. restore the savepoint and validate only the returned columns.

A successful one-sided infeasibility fixing is committed to the same persistent
domain; row and clique consequences are committed with it. Incomplete closure
is not classified as infeasibility and increments a failure counter. There is
no per-world `lb/ub` copy and no per-world scan of all columns.

The direct component regression forces a row implication followed by a
negative-literal clique implication. It requires exactly three returned
columns, full rollback, matching telemetry, and a successful persistent commit.
The production regression requires world/rollback equality, nonzero real trail
and row work, zero failures, and implication-export visits no greater than
changed columns.

### 20.3 Root LP probing contract

Phase 2 retains the separately implemented transaction:

- one `lb/ub` workspace is initialized lazily;
- down/up changes mutate and restore one scalar;
- one synchronized standard form serves all simplex directions;
- `StandardFormBoundTransaction` rolls back touched bound/RHS/objective
  scalars;
- the probe basis drops the node LP's mutable owner before establishing a
  separate persistent owner;
- one-sided infeasibility fixings use
  `update_standard_form_bounds_incremental()`.

These counters describe observed events. None is a synthetic
"copies avoided" value.

### 20.4 Regression evidence

The final focused runs pass:

- `test_branch_and_cut`: 342 assertions in 22 cases;
- `test_milp_solver`: 2,029 assertions in 54 cases;
- the `enlight_hard` production smoke: 204 worlds and rollbacks, 721 trail
  pushes/changed/exported columns, 4,232 processed rows, 16 learned
  implications, and zero failures;
- the same smoke's root LP path: 100 transactions and rollbacks, one cold solve,
  99 persistent resolves, and zero failures.

This establishes execution, sparse extraction, state restoration, and
accounting. It is not general MILP correctness or speed evidence.

### 20.5 Repeated solver comparison

The final report is
`reports/miplib2017_audit_p13_root_domain_probe_transactions_2026-08-02.{json,csv}`.
It uses four instances, three solvers, three repeats, one thread, seed zero, a
3-second backend limit, a 2-second watchdog grace, and relative gap `1e-4`.

Across the 12 Native runs, three execute Phase 1 root domain probing. They
record 3 workspaces, 612 worlds and rollbacks, 2,163 trail pushes, 2,163 unique
changed columns, 2,163 implication-export visits, 12,696 processed rows, 48
learned implications, no committed fixing, and zero failures. On those same
612 `enlight_hard` worlds, the removed unconditional `n=200` extraction loop
would have visited 122,400 columns. This is direct structural-work evidence,
not a wall-time attribution.

Nine Native runs execute Phase 2 root LP probing: 9 workspaces serve 336
directions; all 336 transactions roll back, none fails, and the backend records
12 cold solves plus 324 persistent resolves.

| Solver | Feasible | Proven | Mean fixed-horizon PDI | Shifted PAR-10 |
|---|---:|---:|---:|---:|
| Native B&C + HiGHS LP | 6/12 | 0/12 | 1.984103 s | 30.000 s |
| HiGHS MIP 1.14.0 | 9/12 | 0/12 | 1.676066 s | 30.000 s |
| SCIP MIP 9.0.0 | 9/12 | 3/12 | 1.211046 s | 12.197 s |

Native/HiGHS has paired PDI ratio 1.10202 with 95% interval
`[0.86820, 1.58648]`. Native/SCIP has PDI ratio 4.80789 with interval
`[0.90924, 114.77761]` and PAR-10 ratio 8.61012. All 36 event streams are
available, all 94 primal events pass original-space audit, three transient SCIP
dual events are explicitly discarded, and no process hits the hard watchdog.

The manifest payload SHA-256 is
`c89122b537ae5900dbefcffae9f3833e7edb78eb9e76127aac49dd45e2ab1789`;
the JSON SHA-256 is
`c244bdc3e22a89180b9c53e973df9526dbf760103f1bb5954fc89a25dbc3a74e`;
the CSV SHA-256 is
`c9e7500bfafbb88aa2595719e69268246a745b2ada754f2ff372fb0ade49a27a`.
The manifest records dirty source-state SHA-256
`95819a55b2bdf3897ebfa3fb6f3da5004363959dab32a3e372c16bbadf929d54`,
so this is not a clean-commit artifact.

P13 is structurally closed for the audited root domain mechanism and the
separate root LP copy mechanism. Native still proves 0/12, its PAR-10 is
unchanged, and its mean PDI is effectively unchanged from the earlier report.
There is no end-to-end efficiency improvement evidence. E08 remains `OPEN`;
P11 and P19 remain `PARTIAL`.

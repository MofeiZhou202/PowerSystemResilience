# Native MILP Root-Source Eligibility

## Defect

The transformed root-source loop was gated by the number of continuous
columns whose integrality was inferred during presolve. This excluded ordinary
binary and integer models when that inferred set was empty. On MIPLIB 2017
`sp150x300d`, the root has 271 declared binary columns and zero inferred
integer columns, so the loop did not run.

## Derivation

A tableau split is induced by any fractional integer-constrained basic
variable. Declared integer variables provide these disjunctions directly;
presolve-inferred integer variables only enlarge the eligible set. Therefore
root-source separation is eligible when the root bound is finite and the union
of declared and inferred integer columns is nonempty. The per-round fractional
frontier check remains authoritative and stops the loop when no fractional
integer variable exists.

Reference: T. Achterberg, *Constraint Integer Programming*, PhD thesis, 2007,
Sections 4.1-4.2 (LP-based branch-and-cut and tableau-derived cutting planes).
The lifecycle comparison is HiGHS `HighsSeparator` / `HighsCutGeneration` root
separation.

## Fixed Validation Contract

Baseline commit: `1b51f0b`, macOS ARM64, Release, one thread.

Command:

```text
./tests/miplib2017_benchmark --data-dir tests/data/miplib2017/benchmark \
  --solu tests/data/miplib2017/miplib2017-v36.solu \
  --solvers highs-mip,native-highs-lp --case sp150x300d \
  --time-limit 5 --hard-timeout-grace 2 --max-nodes 5000 --seed 0
```

Prediction: at least one transformed-source round and one additional admitted
cut; reduced root bound at least 20.0 versus 14.90225894; native node count at
most 2226 versus 2969. All reported incumbents must pass original-model audit,
and `test_branch_and_cut` plus `test_milp_solver` must pass.

## First Measurement And Re-Derivation

The first eligibility change opened the outer loop but missed the prediction:
the reduced root bound remained 14.90225894, root cuts remained 124, and the
five-second node count changed only from 2969 to 2926 (1.4%, versus the
predicted 25%). This triggers the mismatch protocol.

1. Implementation infidelity: confirmed. `MIPSOLVERS_XTAB_DIAG=1` reported
   `intOrig=271` and `fracOrig=42`, proving that declared binary metadata and a
   fractional tableau frontier were present. However,
   `generate_root_tableau_cutpool()` independently returned when
   `root_implied_integer_count == 0`, and returned again when no continuous
   column needed re-marking. Those checks contradict the derived union of
   declared and inferred integer columns.
2. Machine/cost-model error: not reached; the intended separator was not run.
3. Assumption violation: not reached; the trace verified the stated integer
   and fractionality assumptions.
4. Theory error: not indicated by this measurement.

The correction is to retain `implied_marked` as diagnostic augmentation only;
the already-declared integer types in the simplex form remain eligible sources.

## Second Implementation-Fidelity Finding

After removing the inner inferred-integrality guards, the transformed-source
trace still reported zero tableau and path calls. The outer activation also
required `auto_highs_root_pipeline`, whose default is false in ordinary native
B&C. That option is documented as the optional root-incumbent pipeline
(analytic-centre, line-search rounding, and lock-count rounding), so it is not
an algorithmic precondition for separating cuts from the already-available
HiGHS LP root state. The MIPLIB native benchmark consequently never reached
the repaired source eligibility test.

This is the same mismatch category: implementation infidelity. The correction
is to run the non-strict transformed-source separator whenever cuts are
enabled, the HiGHS presolved-side state is first-class, the root bound is
finite, and at least one declared or inferred integer column exists. The
`strict_highs_root_fixed_point` exclusion remains: StrictHiGHS owns that root
lifecycle through its direct `evaluateRootNode` path, so running both would
duplicate separation. The optional incumbent/heuristic pipeline remains
controlled by `auto_highs_root_pipeline`.

The cost model and validation thresholds above do not change. This correction
activates the separator that those predictions already assumed was running.

## Second Measurement And Re-Derivation

With both implementation-fidelity corrections, `sp150x300d` ran transformed
tableau and path separation, admitted root cuts, and raised the reduced root
bound from 5.136438176 to approximately 37.001. This exceeds the predicted
20.0 bound. The five-second solve explored 2545 nodes, down 14.3% from 2969,
but missed the predicted maximum of 2226 nodes (25% reduction), so the node
prediction triggers a second mismatch review.

1. Implementation infidelity: the intended separator is now confirmed active
   by source-round, generated-cut, admitted-cut, and re-solve traces. The
   benchmark reported 356 cuts and the original-model incumbent audit passed.
2. Machine/cost-model error: the fixed prediction treated root-bound lift as
   translating directly to nodes per fixed wall time. Added root rows make
   every tree LP larger, while node count is a throughput-weighted metric; the
   measured node reduction therefore combines pruning and lower node
   throughput and cannot identify either effect alone.
3. Assumption violation: HiGHS's root bound is still stronger. In reduced
   objective space, native reached approximately 37.001 while HiGHS reached
   39.703, so the prediction's implicit assumption that activating this
   separator would recover most of the upstream root closure was false.
4. Theory error: not indicated for source eligibility. The result establishes
   that declared integers must be admitted and materially strengthen the root,
   but does not imply equivalence to HiGHS's complete separator portfolio and
   cut-selection policy.

No round-limit or tolerance change follows from this mismatch: those would be
new algorithmic hypotheses requiring their own derivation and benchmark
contract.

## Root-Budget Contract

Broadening the check to `neos-1122047` exposed a third implementation-fidelity
defect. The newly reachable transformed-source loop did not check the common
optional-root deadline between rounds. On a five-second solve it continued
inside root separation until the benchmark's seven-second hard watchdog,
whereas the ordinary root-cut loop checks
`bc_optional_root_budget_expired()` before every round.

The transformed loop is an anytime cutting-plane algorithm: after every
accepted-and-resolved round, its current LP is a valid relaxation and is a
safe stopping state. It must therefore use the same between-round deadline,
including the existing 10% finalization reserve (bounded to 0.05-1.0 seconds).
The predicted interruption granularity is one observed subsecond round;
`neos-1122047` and `p200x1188c` must return native results within 5.5 seconds
instead of reaching the seven-second process watchdog. A weaker root bound is
acceptable only when the global budget is exhausted.

The first budget correction (a check between transformed-source rounds) did
not change `neos-1122047`: a one-second diagnostic showed that the solver never
reached its root LP. PaPILO performed more than 100 fast rounds and millions of
bound/side/coefficient transactions before the external three-second watchdog
terminated the process. The transformed separator was therefore not the
overrun source on this instance. The between-round check remains correct and
consistent with the ordinary root loop, but the predicted 5.5-second return
cannot be attributed to it.

## Production Presolve Replacement

For native B&C with the production HiGHS LP backend, use HiGHS MIP presolve and
retain the exact `HighsPostsolveStack` rather than entering PaPILO. Native B&C
may adopt the reduced model only when the same presolve object provides:

1. reduced integrality and reduced-to-original column identities;
2. an exact affine forward map for every surviving reduced column;
3. the reduced objective offset; and
4. primal postsolve followed by an original-model feasibility audit.

HiGHS defines each surviving affine transformation as
`x_original = scale * x_reduced + constant` (`HighsPostsolveStack::linearTransform`),
so warm-start projection uses
`x_reduced = (x_original - constant) / scale`. Columns marked non-linearly
transformable reject forward projection rather than fabricating a value.

Reference: HiGHS `Highs::presolve`, `Highs::getPresolvedLp`,
`Highs::postsolve`, and `HighsPostsolveStack` linear transformation/postsolve
contracts.

Prediction: under a five-second solve limit, HiGHS presolve on
`neos-1122047` either completes or returns by 1.5 seconds instead of exceeding
the seven-second watchdog. `sp150x300d` must explore no more than 2800 nodes
(10% above the repaired 2545) in five seconds. Every reported incumbent and
incumbent timeline event must pass the original-model audit. The production
HiGHS path uses this replacement; the explicitly experimental native LP path
retains PaPILO for controlled kernel comparisons.

## Forward-Map Test Finding

A focused reducible-knapsack test eliminated a fixed column but also marked at
least one surviving column as not linearly forward-transformable. The proposed
test assumption that every ordinary presolved MILP accepts arbitrary original
warm starts was therefore false. This is an assumption violation, not an
implementation mismatch: the specified forward-map contract rejects exactly
this case rather than inventing reduced values. Postsolve reconstruction is
validated independently with a feasible reduced-space point; the synthetic
affine identity test covers the inverse formula and rejection path directly.

## Production-Presolve First Measurement

Fixed command from the validation contract, Release `-O3 -DNDEBUG`, baseline
commit `1b51f0b`, macOS ARM64:

```text
./tests/miplib2017_benchmark --data-dir tests/data/miplib2017/benchmark \
  --solu tests/data/miplib2017/miplib2017-v36.solu \
  --solvers highs-mip,native-highs-lp \
  --case sp150x300d,p200x1188c,neos-1122047 \
  --time-limit 5 --hard-timeout-grace 2 --max-nodes 5000 --seed 0
```

Measured versus predicted:

- `sp150x300d`: 1017 native nodes versus the predicted maximum 2800;
  original-space audit passed.
- `p200x1188c`: returned at 4.505 seconds with an audited incumbent, 75.6%
  gap.
- `neos-1122047`: reached the 7.070-second hard process watchdog versus the
  prediction that HiGHS presolve would complete or return by 1.5 seconds.

The `neos-1122047` wrong-sign return-time result triggers re-investigation.
No separator limit or tolerance is changed until a timed trace locates the
overrun after the retained presolve pass.

The timed trace locates the overrun after root probing. HiGHS presolve took
141 ms, the root LP 606 ms, root cuts 775 ms, and probing ended at 3.043
seconds. `root_tree_setup_budget_expired` was then sampled once and reused
across standard-form construction, propagation-index construction, and native
IPM node/fallback preparation. Thus a stage that consumed the remaining root
budget did not suppress later optional setup stages. This is implementation
infidelity in the anytime deadline contract, not a presolve or machine-cost
mismatch.

The correction rechecks the common optional-root deadline after every complete
tree-setup stage. Each stage leaves the root relaxation and incumbent state
unchanged, so skipping later stages only yields to finalization. The cost model
is one already-started `O(nnz)` stage of interruption granularity. Prediction:
`neos-1122047` returns by 5.5 seconds under the same five-second validation
command; objective and incumbent audit requirements are unchanged.

The stage-boundary correction still reached the 7.084-second watchdog, while
`sp150x300d` remained stable at 1014 nodes. Therefore the prediction's
interruption-granularity assumption was false: one tree-setup stage can itself
consume more than the roughly two seconds remaining after root probing. The
next implementation-fidelity check is whether tree setup redundantly rebuilds
and rescales a standard form already owned by the persistent root LP state.
No additional setup stage is removed or reordered until that ownership
identity is established from the code.

`PersistentLPState::ensure_base_sf()` is the authoritative cut-free,
Ruiz-scaled root standard form, and root probing explicitly invalidates and
rebuilds it after its final bound changes. Tree setup was therefore rebuilding
the same matrix and scaling a second time. Reusing that persistent form and
only applying the current root bounds preserves the standard-form coefficients
and scaling exactly; the predicted tree pivot path is unchanged once tree
processing starts. Together with the between-stage deadline checks, the fixed
return prediction remains 5.5 seconds for `neos-1122047`.

Timeline instrumentation then identified the dominant stage precisely:
standard-form reuse completed in 8.7 ms, propagation indexing in another 1.0
ms, and the primary IPM stage was inactive; eager preparation of the IPM
fallback did not return before the watchdog. The fallback is only consulted
after all simplex recovery attempts fail. `NativeIPMLPAdapter::solve_node_lp`
already defines a complete uncached path when no prepared state exists, and
that solve receives the remaining wall limit. The correction therefore keeps
the identical IPM recovery algorithm but initializes its cached structure only
through an actual fallback solve instead of paying unconditionally before the
tree. Prediction remains return by 5.5 seconds; ordinary successful-simplex
tree paths avoid this setup entirely.

After removing eager fallback preparation, the trace reached the root child
and then remained inside `child_pre_lp` domain closure until the watchdog.
That closure permits up to `2n+8` monotone passes and its row propagation had
no deadline input. The correction polls the shared finalization deadline every
64 processed variables/rows. Interruption preserves all already-derived bound
tightenings but marks the child as incomplete; it is neither counted as a
cutoff nor used for a dual-bound improvement, and the solve terminates with the
certified root bound. Prediction remains return by 5.5 seconds, with polling
granularity below 10 ms on this sparse instance.

The exact three-case validation after this correction measured
`neos-1122047` returning normally in 4.923 seconds with one explored node,
versus the predicted 5.5-second maximum and the prior 7.0-second watchdog.
`sp150x300d` explored 1041 nodes versus the predicted maximum 2800, and both
native incumbents available in the run passed original-space audit.

## Twenty-Four-Case Sample

The fixed 24-instance sample completed with all reported incumbents passing
original-space audit: HiGHS 13/13 and native 3/3. Native solved none to proven
optimality within five seconds, versus three for HiGHS. The targeted cases
remained stable (`neos-1122047` returned in 4.979 seconds; `sp150x300d` used
1037 nodes), but native still reached the external watchdog on
`neos-3627168-kasai`, `neos-950242`, and `rmatr200-p5`. Several additional
models returned `Root relaxation failed`. These are residual native root/tree
deadline or LP robustness defects, not evidence that the production presolve
replacement failed: no PaPILO pass runs on the production HiGHS LP backend,
and the retained HiGHS presolve/postsolve mapping passed every available
incumbent audit.

## Root-Gap Coordinate Audit

Final semantic review found that the verified warm-start shortcut compared
`heuristic_obj` in the original internal-minimization objective space with
`root.bound` in the presolved space. When presolve removes an objective
constant, HiGHS defines

```text
f_original = f_reduced + presolved_objective_offset.
```

The certificate comparison must therefore use
`heuristic_obj - presolved_objective_offset`. The same identity applies to the
retained PaPILO path. Without this conversion, a sufficiently large nonzero
offset can make the shortcut report `OptimalRootGapClosed` even though the
actual reduced-space gap exceeds the requested tolerance.

Reference: HiGHS `HighsLp::offset_` and the objective identity used by
`Highs::getPresolvedLp`; PaPILO's retained `objective_offset` contract.

Prediction: this O(1) coordinate conversion has no measurable effect on the
fixed MIPLIB validation cases unless a verified warm start with a nonzero
presolve offset reaches the shortcut. A focused regression with a deliberately
nonzero offset must no longer return `OptimalRootGapClosed`; both native MILP
test suites and the fixed three-case benchmark must retain their existing
acceptance results.

The first behavioral fixture missed this prediction: it still returned the
suboptimal warm start as `OptimalRootGapClosed`. Implementation tracing showed
that the target HiGHS-reduced path was not active; HiGHS did not adopt the
fixture's reduced model, and the subsequent native presolver removed the fixed
column (`rows 1->1`, `cols 4->3`). Native presolve does not expose an objective
offset, so this exercised the pre-existing native-coordinate issue rather than
the retained HiGHS correction. This is a validation-fixture assumption
violation, not measured evidence against the objective identity. The fixture
must first assert `HighsLpPresolveResult::use_reduced` and a nonzero retained
offset before it is used to validate the B&C shortcut.

Adding the fixed column to the single covering row still did not produce an
adoptable reduced model: HiGHS can fully resolve this tiny covering instance
during presolve. This is the same fixture assumption violation. The corrected
regression uses the existing nontrivial knapsack fixture, for which retained
HiGHS reduction is independently established, augmented by a fixed objective
column and a feasible but suboptimal warm start.

That fixture activates the intended path and reports reduced root bound `-33`
after removing the `+100` fixed-column offset. The corrected shortcut rejects
the warm start; root processing then finds the integral optimum (`133` in the
original maximize direction). The solver still uses the shared
`OptimalRootGapClosed` status for this later legitimate proof, so status alone
cannot distinguish the shortcut. The regression therefore checks that the
published objective strictly improves on the `100` warm start.

The semantic audit also requires the standard relative-gap normalization to
remain in original objective space: the offset cancels only in the numerator,
so the denominator is `max(1, abs(heuristic_obj))`, not the reduced objective
magnitude. On an early successful return, public `best_obj` and `best_bound`
must likewise use the caller's objective direction; the internal root bound is
first shifted by the offset and then sign-restored for maximization.

The final fixed three-case benchmark after the certificate corrections measured
`neos-1122047` returning normally in 4.907 seconds with one explored node,
`sp150x300d` exploring 1035 nodes, and `p200x1188c` returning in 4.504 seconds.
Both available native incumbents passed original-space audit, preserving the
existing acceptance thresholds.

## Reduced-To-Empty Certificate

Final status review found that the production B&C caller retained
`HighsLpPresolveResult::solved_by_presolve` but did not consume it. A
`ReducedToEmpty` HiGHS result has no remaining branching or LP variables; the
empty reduced primal passed through the same retained postsolve stack yields
the original primal certificate. Publishing optimality is valid only after
that reconstructed primal passes the existing original-model residual audit.
If recovery or audit fails, B&C continues on the untouched original model.

Reference: HiGHS `HighsPresolveStatus::kReducedToEmpty`, `Highs::postsolve`,
and the retained `HighsPostsolveStack` contract already used by the native LP
kernels.

Prediction: an audited reduced-to-empty fixture returns optimal with zero B&C
nodes and zero root LP solves; instances with a nonempty reduced model are
unchanged. Cost is the completed HiGHS presolve plus one O(nnz) original-model
audit. The two native MILP suites and fixed three-case benchmark must preserve
their existing acceptance results.

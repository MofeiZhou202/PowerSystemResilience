# Native MILP system theory, external progress, and full improvement plan

Date: 2026-08-01  
Status: architecture, truthfulness audit, and first controlled experiment complete;
P1-P7 performance roadmap remains gated by the evidence below  
Scope: linear MILP, the native branch-and-cut tree, its LP kernels, and the
strict HiGHS/SCIP reference paths

## 1. Executive conclusion

The current evidence does **not** show that Native MILP is generally faster
than HiGHS or SCIP. It now shows one reproducible end-to-end gain after a
deadline-state bug was removed: on `neos-3083819-nubu`, Native B&C with the
HiGHS node-LP kernel proved optimality in all three runs (2.030-2.066 s), while
HiGHS and SCIP did not close the requested gap within 3 s. Across the full
four-instance cohort, SCIP still had the best shifted PAR-10 (12.143 s),
followed by Native B&C plus HiGHS LP (16.353 s); HiGHS and the experimental
native LP path both scored 30 s. Four selected instances are not a general
solver-performance claim.

Reliability branching separately reduced search work on `enlight_hard`, but
that path still exhausted its queue without a publishable incumbent. This
remains a correctness/publication gap, not a solve. The experimental native LP
kernel recovered audited incumbents on `50v-10`, but proved no runs and ignored
the soft deadline badly enough to hit the process watchdog on all three `neos`
runs.

The main bottleneck is not the absence of one more branching score. It is the
whole chain:

1. benchmark truth and hard deadlines;
2. original-space correctness and postsolve contracts;
3. presolve/domain parity;
4. root and node LP reliability;
5. warm node-LP throughput;
6. cut and primal-heuristic portfolio control;
7. search, restart, and parallel coordination.

On the large SCUC evidence already in this repository, the native LP tree
processed about 10 node LPs in roughly 150 seconds while HiGHS processed orders
of magnitude more. Eliminating all branching overhead would not close that
gap. LP reoptimization is therefore ahead of additional branching tuning in
the dependency order.

The production conclusion is correspondingly narrow:

- use strict HiGHS or SCIP when a competitive general MILP solve is required;
- use Native B&C as an experimental research path with independent audits;
- do not describe a strict HiGHS solve as Native algorithmic performance;
- do not promote a local node-count reduction to an end-to-end speed claim.

## 2. Mathematical objective and truthful metrics

Consider the minimization form

$$
  \min\ c^T x + c_0
  \quad\text{s.t.}\quad
  \ell_r \le Ax \le u_r,\quad
  \ell_x \le x \le u_x,\quad
  x_j \in \mathbb Z\ (j \in I).
$$

At time $t$, let $P(t)$ be the best audited incumbent and $D(t)$ the best
valid global dual bound. For minimization, $D(t) \le z^* \le P(t)$. A normalized
primal-dual gap is

$$
g(t)=\frac{\max(0,P(t)-D(t))}
           {\max(1,|P(t)|,|D(t)|)}.
$$

If either bound is unavailable, the gap is unavailable. It is not zero and
must not be synthesized from node counts. The primal-dual integral over limit
$T$ is

$$
  \operatorname{PDI}(T)=\int_0^T \min(1,g(t))\,dt,
$$

with an explicit penalty convention before the first incumbent. PDI measures
the quality of the entire solve trajectory, unlike final gap or wall time
alone. The benchmark must report at least:

- independently audited optimal/proven counts;
- wall time and a hard-timeout flag;
- shifted geometric mean
  $\exp(\frac1n\sum_i\log(t_i+s))-s$;
- PAR-10, with unsolved runs charged $10T$;
- PDI, primal integral, and time to first audited incumbent when callbacks make
  those observations available;
- nodes, LP solves, LP iterations, cuts, and strong-branch work only as
  diagnostic counters with explicit availability.

Three repetitions are the minimum for deterministic-looking comparisons.
Paired differences and a paired bootstrap confidence interval are required
before claiming a gain. Node count alone is never an acceptance metric because
strong branching, separation, and heuristics change the cost per node.

## 3. HiGHS 1.14 MIP theory and source state machine

This section is based on the vendored HiGHS 1.14 source, not only public option
names. The local `highs/` tree contains project-specific `hacdcpf` tracing and
experimental changes, so it is not pristine upstream 1.14.

### 3.1 Top-level lifecycle

The entry is `HighsMipSolver::run()` in
`highs/mip/HighsMipSolver.cpp:85`. The effective lifecycle is:

1. **Model normalization and setup.** Normalize objective sense and offsets,
   validate integrality and bounds, initialize clocks, limits, and callbacks.
2. **MIP presolve.** `HighsMipSolverData::runMipPresolve()` at
   `HighsMipSolverData.cpp:1351` builds the reduced model and postsolve stack.
3. **Core state construction.** `runSetup()` at line 1440 constructs the
   domain, clique table, implication store, cut/conflict pools, pseudocosts,
   node queue, objective structure, and propagation machinery.
4. **Structural analysis.** Extract cliques and implications, detect implied
   integers, analyze symmetry/orbitopes, and optionally compute an analytic
   center used by heuristics and branching signals.
5. **Root LP.** Install the root domain and solve the relaxation with a basis-
   producing LP algorithm suitable for repeated reoptimization.
6. **Root separation fixed point.** `rootSeparationRound()` at line 2468 and
   `evaluateRootNode()` at line 2610 alternate separation, propagation,
   reoptimization, cut aging, and heuristic calls.
7. **Root restart.** `performRestart()` at line 1948 can rebuild the reduced
   problem after incumbent/domain progress changes the useful formulation.
8. **Tree search.** Install an open node, apply bound changes, propagate,
   evaluate the node LP, separate cuts, run heuristics, branch, dive, and
   return deferred children to the global queue.
9. **Termination and postsolve.** Convert the incumbent and dual bound through
   objective offset/sense and presolve mappings; publish only statuses justified
   by the final global state.

This is a feedback system, not a linear list of features. A new incumbent
changes cutoff propagation, cut usefulness, node pruning, pseudocost samples,
and restart decisions. That is why isolated branching changes can reduce nodes
without reducing total time.

### 3.2 Search and reliability branching

`HighsSearch::selectBranchingCandidate()` starts at
`highs/mip/HighsSearch.cpp:307`. For a fractional variable $j$ with fractional
distances $f_j^-$ and $f_j^+$, pseudocost predictions are

$$
 \hat g_j^- = \psi_j^- f_j^-, \qquad
 \hat g_j^+ = \psi_j^+ f_j^+.
$$

HiGHS tracks reliability separately by direction. Unreliable competitive
candidates receive capped strong-branch LP probes. Probe outcomes update local
scores and global pseudocost/inference/cutoff observations. Candidate selection
is repeated as information arrives, so an initially strong candidate can be
displaced. Degeneracy factors and inference history alter tie breaking.

Node evaluation is `HighsSearch::evaluateNode()` at line 951, branching is
`HighsSearch::branch()` at line 1409, and depth-first plunging is
`HighsSearch::dive()` at line 2206. The global queue and the dive are
complementary: the dive exploits a locally promising path while the queue
preserves global bound progress and recovery from a bad plunge.

### 3.3 Domain, conflict, and clique feedback

The important state is shared across modules:

- `HighsDomain`: current/global bounds, propagation stack, infeasibility;
- `HighsCliqueTable`: set-packing structure, binary products, substitutions;
- `HighsImplications`: bound implications and implied-bound cuts;
- `HighsConflictPool`: learned conflicts with aging;
- `HighsCutPool`: global candidate cuts, violation selection, aging;
- `HighsPseudocost`: directional objective gain, inference, cutoff history;
- `HighsNodeQueue`: open-node bound and estimate ordering;
- symmetry/orbitope state: reductions that avoid equivalent subtrees.

Propagation is incremental. A changed bound activates affected rows and
implications; an infeasible implication chain can create a conflict; the
conflict can tighten another domain or enter a pool; those tightenings change
the next LP and branching signals. Replacing this with repeated full row scans
loses both speed and learning.

### 3.4 Separation and cut aging

`HighsSeparation::separationRound()` begins at
`highs/mip/HighsSeparation.cpp:73`. In the vendored flow, implied-bound and
clique separation/propagation precede the general transformed-LP separators.
Candidates enter a cut pool, are selected at the current LP solution, appended
to the relaxation, and then reoptimized. `performAging(true)` removes stale LP
rows, while the cut pool has its own aging.

A useful cut must satisfy four conditions simultaneously:

1. valid in original integer space;
2. sufficiently violated after numerical margins;
3. not too dense, dynamic, or parallel to selected rows;
4. worth its added LP solve and factorization cost.

Therefore "the family generated valid cuts" is not a performance result.
Selection and aging are part of the algorithm.

### 3.5 Primal heuristic portfolio

`HighsPrimalHeuristics.cpp` coordinates randomized rounding/shifting,
feasibility pump style work, RENS/RINS neighborhoods, and sub-MIPs. HiGHS also
contains the LP-free Feasibility Jump implementation in
`HighsFeasibilityJump.cpp`. Heuristics are scheduled by depth, incumbent state,
stalling, and budget. A heuristic is valuable when improvement/pruning saved by
its incumbent exceeds the LP/sub-MIP budget it consumed.

## 4. Upstream changes the local fork does not yet contain

### 4.1 HiGHS 1.14 to 1.15.1

The official latest release checked on 2026-08-01 is HiGHS 1.15.1, published
2026-07-02. Its release notes identify these relevant changes:

- first prototype multithreaded MIP solver, with worker/node-search refactored
  to run across threads;
- a parallel scheduler fix and removal of thread-sanitizer race warnings;
- parallel simplex disabled for LP relaxations inside parallel MIP to avoid
  unsafe nested parallelism;
- strengthened bounds found by presolve are retained;
- fixes for semi-continuous/semi-integer infeasibility and implied-bound
  rechecking;
- `Highs::setBasis` and `setLogicalBasis` implementations;
- HiPO exposed to Python, with faster free-variable and triangular-solve work;
- OpenBLAS performance setup fixes in 1.15.1.

The local strict HiGHS path is 1.14 and its MIP tree is serial. Passing
`threads=N` to it must not be reported as $N$ effective MIP workers. This audit
now reports one effective MIP thread and marks unavailable internal counters as
unknown.

Upgrading cannot be a blind directory replacement because the local fork has
`hacdcpf` root ledgers, callbacks, conformance traces, cut projection, and other
changes. Upstream 1.15.1 must first be built as a separate differential oracle;
then local changes can be rebased or removed one contract at a time.

### 4.2 SCIP 9.0 to 10.0.3

The local vendored SCIP is 9.0.0. The official current release checked here is
SCIP 10.0.3, published 2026-07-06. SCIP 10.0 introduced:

- exact rational MILP solving without floating-point feasibility tolerances;
- implied-integrality presolving;
- reflection-symmetry handling;
- cut-based conflict analysis;
- improved branching and two decomposition-aware primal heuristics;
- improved Benders decomposition, IIS tools, and JSON statistics.

These are not feature-checklist decorations. Exact solving changes the proof
contract; cut-based conflict analysis connects LP infeasibility with learned
global constraints; reflection symmetries generalize permutation-only
reductions. SCIP 10 should be a second external oracle even if it is not the
production backend.

## 5. Verified MILP research directions

The following items were checked against publisher/Crossref metadata or
official solver releases. They are grouped by what they change in the solve,
not by novelty.

| Direction | Verified source | Engineering consequence |
|---|---|---|
| Reliability/hybrid branching | Achterberg, Koch, Martin, *Branching rules revisited*, 2005, DOI `10.1016/j.orl.2004.04.002` | Strong branch only while directional estimates are unreliable; count probe cost. |
| Parallel/multiprecision presolve | Gleixner et al., *PaPILO*, 2023, DOI `10.1287/ijoc.2022.0171` | Transactional reductions and exact postsolve maps are more important than the number of presolvers. |
| Conflict learning | Achterberg, *Conflict analysis in MIP*, 2007, DOI `10.1016/j.disopt.2006.10.006` | Learn from propagation and LP infeasibility with globally valid reasons. |
| Cut-based conflict analysis | Mexi et al., 2025, DOI `10.1287/ijoc.2024.0999`; SCIP 10 | Derive reusable inequalities, not only bound-disjunction clauses. |
| Adaptive cut selection | Turner et al., 2023, DOI `10.5802/ojmo.25` | Score efficacy, directed cutoff distance, density, parallelism, and downstream LP cost jointly. |
| LP-free primal search | Berthold, Mexi, Salvagnin, *Feasibility Jump*, 2023, DOI `10.1007/s12532-023-00234-8` | Add a cheap complementary incumbent source; budget by integral improvement, not call count. |
| General symmetry handling | van Doornmalen and Hojny, 2024, DOI `10.1007/s10107-024-02102-2` | Unify orbital reductions, orbitopes, and reflection symmetries behind valid stabilizer updates. |
| Learning to branch | Gasse et al., NeurIPS 2019; Scavuzzo et al., *Learning to Branch with Tree MDPs*, 2022, DOI `10.52202/068431-1345` | Learned scores need uncertainty/OOD gates and deterministic classical fallback. |
| Learning cut decisions | Paulus et al., *Learning to Cut by Looking Ahead*, ICML 2022 | Training labels must include downstream LP/tree cost, not immediate violation only. |
| Exact/proof-oriented MILP | SCIP 10 official release, 2025-2026 | Status `Optimal` should become independently checkable for high-assurance modes. |
| First-order/GPU LP kernels | Applegate et al., *PDLP*, 2026, DOI `10.1007/s12532-026-00309-2`; HiGHS 1.15 HiPO | Useful at very large roots or as a portfolio seed, but tree nodes still require a robust basis/crossover path. |
| Benchmark methodology | Gleixner et al., *MIPLIB 2017*, 2021, DOI `10.1007/s12532-020-00194-3` | Use an immutable heterogeneous corpus and data-driven metrics; never tune on a named instance. |

Recent machine learning work is conditional, not the first implementation
priority. A learned decision layer cannot repair invalid postsolve, a singular
warm basis, or a node LP that is two orders of magnitude too slow.

## 6. Native versus HiGHS/SCIP gap matrix

`Implemented` below means code exists. It does not mean performance parity.

| Subsystem | Current Native evidence | HiGHS/SCIP reference | Gap and decision |
|---|---|---|---|
| Status, deadline, postsolve | Failure/status scrubbing, original-space benchmark audits, and POSIX child-process hard deadlines exist. Generic strategy presolve is now an honest identity facade. | Mature presolve/postsolve stacks and limit handling. | Windows still has only the backend soft limit; proof-grade status validation remains missing. P0/P1. |
| Presolve/domain | PaPILO MIP presolve, event propagation, clique/implication code exist. | Integrated repeated presolve, domain, implication, clique and symmetry feedback. | Reduction quality and postsolve parity are not established on full MIPLIB. P3. |
| Root LP | HiGHS LP backend is usable; native dual/IPM paths exist. | Mature dual simplex/IPM/crossover. | Native presolved roots and degenerate large LPs remain fragile; IPM is not a reliable default. P4. |
| Node LP reoptimization | Warm basis and fallback machinery exist with extensive diagnostics. | Factor/basis repair and highly optimized FTRAN/BTRAN/repricing. | Dominant gap: singular warm bases and very high per-pivot/per-LP cost. P4 before more tree tuning. |
| Cuts | GMI, covers, cliques, implied bounds, transformed cuts and experimental CGLP paths exist. | Integrated selection, numerical safeguards, pool/LP aging and broad separator portfolio. | Family presence exceeds evidence. CGLP stays off; dead disabled MIR implementation was removed. P5. |
| Primal heuristics | Several rounding/repair/local mechanisms and subsolve counters exist. | Feasibility Jump, RENS/RINS, randomized/shifting, sub-MIP scheduling. | No portfolio-level primal-integral ablation. P5. |
| Branching | Reliability probing, directional pseudocosts, exact-result reuse and calibration telemetry now exist. | HiGHS reliability branching and SCIP hybrid strategies are mature. | One local search-work win; no aggregate wall-time win. Freeze knobs until P4/P5. |
| Queue/restart | Native queue, estimates and restart-related logic exist. | Joint queue/dive/restart system with global domain/cut feedback. | No paired evidence that Native estimates improve PDI or time to proof. P6. |
| Conflict/clique/symmetry | Clique table, implication and proof-oriented conflict mechanisms exist; strict path can use HiGHS symmetry. | HiGHS integrated clique/implication/symmetry; SCIP 10 cut conflicts/reflections. | Native global-validity and payoff are not established. P3/P5. |
| Parallelism | Experimental parallel tree/shared state exists. | HiGHS 1.15 prototype parallel MIP; SCIP concurrent/parallel modes. | No scalable speedup or deterministic/TSAN acceptance evidence. P6. |
| Exact certificates | Some proof artifacts and audits exist. | SCIP 10 exact rational mode. | No end-to-end independently checkable Native optimality certificate. P1, then conditional exact mode. |
| Observability | Large `BCStats`, JSON and MIPLIB runner exist. Availability flags distinguish unknown from zero; POSIX runs have a process watchdog. | Solver-native event streams and established benchmark harnesses. | PDI, paired CI, randomized blocked order and immutable run metadata remain missing. P0. |

## 7. False-capability and placeholder audit

The 2026-08-01 audit made these behavioral changes:

- removed unused `void*` IPM state from parallel shared state;
- made warm-start components and callbacks either work or throw explicit
  `invalid_argument`; unsupported state is no longer silently discarded;
- implemented primal hints and MINLP initial points with dimension/finite
  checks, while rejecting unsupported dual/basis/cut-pool restart payloads;
- replaced the generic `PresolveManager` claims with an explicit identity
  compatibility facade; real MIP presolve remains `papilo_presolve_mip()`;
- made `PostsolveManager` validate mappings, scaling, finiteness, dimensions,
  and objective consistency;
- implemented AML LP/MPS export and parameter-storage validation;
- AML now rejects MIQP/MINLP instead of silently discarding a quadratic
  objective or integer restrictions, and preserves linear objective constants;
- removed the disabled duplicate MIR implementation instead of leaving it
  behind `#if 0`;
- corrected CGLP documentation: it is an off-by-default experimental solved
  CGLP/fallback path, not a no-op;
- removed fabricated strict-HiGHS statistics (`lp_solves = nodes + 1`, inferred
  incumbent timeline, requested threads reported as effective MIP threads);
- added statistics provenance/availability and changed uncomputed maximum
  clique size from integer zero to `optional<int>`.
- made the MIPLIB runner isolate every solver invocation on POSIX, terminate it
  after `limit + grace`, and distinguish backend soft limits from hard process
  timeouts;
- made unavailable MIPLIB counters and diagnostics JSON `null` and CSV empty,
  including strict-HiGHS branching/fallback fields that previously looked like
  observed zeros.

The scoped marker scan is reproducible and intentionally ownership-aware:

- first-party `include/`, `src/`, `benchmark/`, and `tests/`, excluding the
  standalone vendored HFactor copy: **0** TODO/FIXME/HACK/placeholder/fake/stub/
  unimplemented markers;
- standalone vendored HFactor copy: **8** upstream markers or compatibility
  descriptions, retained and classified as unaccepted upstream debt;
- CMake integration: **17** `stub` references, all naming real Fortran-main,
  sequential-MPI, or SCIP optional-plugin compatibility targets. Removing or
  renaming these would hide their purpose and can break linking; they are not
  counted as solver features.

Compatibility code is not solver capability. The CHOLMOD-disabled class
returns false so callers select another backend; compile-disabled Gurobi
reports unavailable; MUMPS `mpiseq` is a sequential MPI compatibility layer;
the standalone HFactor `HConfig.h` only configures that port. None of those may
be counted as an active solver implementation.

One explicit HFactor debt remains: the vendored standalone port preserves an
upstream warning around the hyper-sparse iTran lookup, and local FT-update
behavior differs from the complete HiGHS copy. The port is not accepted as a
cold-start or production factor backend until dedicated FTRAN/BTRAN/update and
rank-repair tests pass.

## 8. Dependency-ordered implementation plan

### P0. Benchmark truth and hard deadlines

Deliverables:

- **Implemented on POSIX:** run every solver invocation in an isolated child
  process;
- **Implemented on POSIX:** terminate the child after `limit + grace`, record
  `hard_timeout=true`, and
  never block the rest of the corpus on a solver ignoring its limit;
- freeze a manifest of instance path, SHA-256, reference status/objective,
  supported variable types, seed, threads, options, compiler, CPU and commit;
- record time-series incumbent/dual-bound events when actually available;
- **Implemented:** serialize unavailable counters as JSON `null` and empty CSV
  cells, never zero estimates;
- run at least three repetitions in blocked/randomized solver order.

Acceptance gate:

- no process exceeds the hard deadline plus documented grace;
- every published incumbent passes original MPS row, bound, integrality and
  objective-offset audits;
- every summary can be regenerated from raw rows;
- shifted geometric mean, PAR-10, PDI and paired confidence intervals are
  present, or explicitly `null` when the source event stream is unavailable.

### P1. Correctness and proof contracts

Deliverables:

- original-to-reduced mappings for every presolve/fix/substitution;
- exact status state machine for feasible, proven optimal, infeasible,
  unbounded, limit, interrupt and numerical failure;
- cut validity audit against known feasible points plus rational/debug replay;
- differential tests against HiGHS 1.15.1 and SCIP 10 on generated small MILPs;
- optional proof/checker path before claiming exact or high-assurance status.

Acceptance gate:

- zero objective/status disagreements on the accepted oracle corpus;
- zero invalid accepted cuts and zero postsolve feasibility failures;
- no success status with non-finite objective, missing incumbent, or failed
  original-space audit.

### P2. Upstream differential oracles

Build pristine HiGHS 1.15.1 and SCIP 10 beside, not over, the local forks.
Replay models, options, root relaxations, cuts, incumbents and final statuses.
Classify every difference as intended local behavior, upstream fix, or bug.

Acceptance gate: a written patch inventory and automated differential suite;
only then choose which local HiGHS changes to rebase, delete or retain.

### P3. Presolve, domain, clique and symmetry parity

Make PaPILO reductions transactional, preserve strengthened bounds, close the
domain/implication/clique fixpoint, and add symmetry reductions only with a
valid stabilizer/postsolve contract.

Acceptance gate: no oracle disagreement; reduced size/root-bound improvements
on a heterogeneous corpus; no worse shifted-geomean root time outside a paired
confidence interval.

### P4. Root and node LP throughput

This is the highest expected performance lever. Work items are:

- synchronize the standalone HFactor port with upstream and prove FT/PF update
  correctness;
- implement singular-basis repair with logical columns and auditable mapping;
- profile/refactor hyper-sparse FTRAN/BTRAN, DSE updates, pricing and
  refactorization frequency;
- make native BFRT/degeneracy handling robust without starving Phase I;
- treat IPM/HiPO/PDLP as root portfolio candidates only when crossover produces
  a valid warm basis within budget.

Acceptance gate: warm reoptimization failure rate, median/shifted-geomean node
LP time, pivots per bound change, and residual audits all improve on the frozen
node-LP trace corpus. A wall-time tree improvement must follow; a kernel
microbenchmark alone is insufficient.

### P5. Cut and primal-heuristic portfolio

Use a common candidate contract for efficacy, directed cutoff distance,
density, dynamism, parallelism, validity margin and generation/LP cost. Add
Feasibility Jump, RENS/RINS and local branching behind budget controllers.
CGLP remains conditional until it wins a paired ablation.

Acceptance gate: improved root gap/PDI/time-to-first-incumbent and final PAR-10
without audit failures. Every family must have an on/off ablation; no named-
instance parameter tuning.

### P6. Search, restart and parallelism

After node LPs are cheap and stable, unify queue estimates, dive policy,
restart triggers and shared global state. Use HiGHS 1.15's worker/node-search
design as an oracle. Share incumbents, bounds, global cuts and pseudocosts with
clear ownership and deterministic mode.

Acceptance gate: reproducible one-thread results, clean thread sanitizer, and
paired multicore speedup after charging synchronization overhead. Requested
threads and effective workers must be reported separately.

### P7. Guarded learning

Only after P0-P6 generate trustworthy traces, train branching/cut/heuristic
policies. Attach uncertainty and out-of-distribution detection; fall back to
the deterministic classical policy when confidence is low.

Acceptance gate: held-out family/scale tests with paired confidence intervals,
zero correctness-contract changes, bounded inference cost, and no regression
when the model is disabled or rejects an input.

## 9. Rejected and conditional ideas

Rejected by current evidence:

- declaring a win from fewer nodes on `enlight_hard`;
- replacing `Sum` by `Maximum`: Phase F showed no aggregate benefit;
- tuning probe counts/reliability thresholds per named MIPLIB instance;
- aggressive relative pivot floors: prior tests broke the large cold root;
- KLU rescue as the default large-basis repair: prior tests made factorization
  prohibitively expensive;
- adding ML before benchmark truth and node-LP throughput are fixed;
- reporting strict HiGHS internals as Native implementation results.

Conditional, with gates:

- CGLP/disjunctive cuts: only after validity and paired portfolio ablations;
- GPU/PDLP/HiPO root race: only with bounded crossover and a valid basis;
- exact rational mode: after a checker/proof artifact contract exists;
- parallel tree: after single-thread state is reproducible and TSAN-clean;
- decomposition/Benders: for identified decomposable families, not as a
  general-MILP replacement.

## 10. Numerical evidence and current interpretation

### 10.1 Reliability-branching experiment

The reliability-branching experiment used a fixed cohort of `50v-10`,
`enlight_hard`, `neos-3083819-nubu`, and `wachplan`, one thread, seed zero,
10-second limits, three repetitions. Raw reports are:

- `reports/miplib2017_reliability_phasef_sum_repeat3_2026-08-01.json`;
- `reports/miplib2017_reliability_phasef_maximum_repeat3_2026-08-01.json`.

Observed facts:

- `enlight_hard` search work fell from the earlier 262 nodes / 991 LP solves
  to 120 nodes / 819 LP solves in the final reliability run;
- this is a local algorithmic signal, not an overall speed result;
- `Sum` and `Maximum` both proved only 3 of 12 repeated runs in the fixed
  cohort and had essentially identical shifted PAR-10;
- the broader 12-case native-native-LP sample mostly failed at the root, so it
  cannot rank branching policies;
- `enlight_hard` ended with an exhausted queue but no publishable incumbent in
  the cited report, while the reference optimum is known. That is a result-
  publication/correctness gap, not a success.

That experiment alone supports only this conclusion: **reliability branching
reduced local search work on one case, while its end-to-end efficiency gain was
not established.**

### 10.2 Truthfulness fix and controlled 3-second experiment

The system audit then exposed two separate false-timeout paths. First,
`solve_lp_relaxation()` relabeled any failed native simplex root as `Time limit`
whenever the assigned LP budget was at most 10 seconds. Second, the presolved-
root fallback reserve used

$$
  \max(T_{\min}^{\mathrm{fallback}}, \rho T)
$$

even when the entire global limit was below
$T_{\min}^{\mathrm{fallback}}$. With the defaults $T=3$ s,
$T_{\min}^{\mathrm{fallback}}=5$ s and $\rho=0.5$, this collapsed the root-LP
budget to 1 ms while simultaneously making the fallback ineligible. On
`50v-10`, the experimental native LP path therefore returned a false global
timeout after 6-14 ms without an incumbent.

The unconditional relabel was deleted. Root budgeting now reserves fallback
time only when the fallback eligibility threshold is reachable; otherwise the
root owns the remaining global budget. The budget rule is covered by a pure
unit test, and root statuses accept the detailed `Time limit: ...` prefix. A
post-fix smoke run used 3010.9 ms, explored 143 nodes, executed 365 LP solves,
and returned an incumbent that passed the original-MPS audit. This is a
correctness and effective-work recovery, not an optimality proof.

The final controlled experiment used the same four instances, four explicitly
named solver paths, one thread, seed zero, a 3-second backend limit, a 2-second
process-watchdog grace, and three repetitions. Every published incumbent was
audited against the original MPS model. Raw rows and summaries are in:

- `reports/miplib2017_system_audit_2026-08-01.csv`;
- `reports/miplib2017_system_audit_2026-08-01.json`.

| Solver path | Runs | Audited incumbents | Proven | Hard timeouts | Shifted PAR-10 |
|---|---:|---:|---:|---:|---:|
| HiGHS MIP | 12 | 9 | 0 | 0 | 30.000 s |
| SCIP MIP | 12 | 9 | 3 | 0 | 12.143 s |
| Native B&C + HiGHS LP | 12 | 6 | 3 | 0 | 16.353 s |
| Native B&C + native LP | 12 | 3 | 0 | 3 | 30.000 s |

Per-instance facts that materially qualify the aggregate:

- SCIP proved `enlight_hard` in all repeats in 1.71-1.79 ms. Native B&C plus
  HiGHS LP used 120 nodes and 819 LP solves in about 0.55 s but published no
  incumbent and no proof.
- Native B&C plus HiGHS LP proved `neos-3083819-nubu` in every repeat in
  2.030-2.066 s with one explored node and 62 LP solves. The original-space
  incumbent audit passed. HiGHS ended at gap 0.000865-0.000889 and SCIP at
  0.00224-0.00338 under the same 3-second limit.
- Native B&C plus native LP produced audited `50v-10` incumbents in every
  repeat, with 139-142 nodes and final gap about 0.490, but no proof.
- Native B&C plus native LP exceeded the 3-second soft limit on
  `enlight_hard` (about 3.70 s) and hit the 5-second hard watchdog on all three
  `neos` runs. Native B&C plus HiGHS LP also overran the backend limit on
  `wachplan` (about 4.28 s). Deadline polling inside long root/subsolve stages
  is therefore still a P0 defect even though the parent process now contains
  it.

The sequential tree was subsequently audited at the LP call boundary. The
current remaining budget and a shared deadline flag now reach the main node
LP, reliability probes, dispatcher, and all twelve direct certificate/refresh
re-solves; a timed-out primary solve no longer enters the fallback chain or a
second child direction. The `neos` native-LP reproduction nevertheless still
reaches the hard watchdog. Timeline traces stop inside one node evaluation
after domain closure and before the next LP result is returned. The remaining
P0 defect is therefore narrower but more fundamental: a single native
factorization/numerical operation can remain inside a non-preemptible kernel
call beyond the deadline. Process isolation remains required until that kernel
operation is decomposed or made interruptible.

The defensible conclusion is: **the truthfulness fixes restored real search
and produced a repeatable, instance-specific end-to-end proof improvement for
Native B&C plus HiGHS LP. They did not establish general Native MILP efficiency
parity. SCIP still wins this cohort; the experimental native LP kernel remains
noncompetitive and has a confirmed soft-deadline defect.**

The next general publishable performance conclusion still requires the
immutable manifest, blocked repetitions, paired confidence intervals and PDI
event stream. The hard-deadline and null-statistics pieces alone do not supply
that evidence.

## 11. Sources

1. HiGHS v1.15.1 official release, 2026-07-02:
   <https://github.com/ERGO-Code/HiGHS/releases/tag/v1.15.1>.
2. HiGHS project background and LP citation:
   <https://highs.dev/>; Huangfu and Hall, DOI
   `10.1007/s12532-017-0130-5`.
3. SCIP v10.0.0 and v10.0.3 official releases:
   <https://github.com/scipopt/scip/releases/tag/v10.0.0> and
   <https://github.com/scipopt/scip/releases/tag/v10.0.3>.
4. Achterberg, Koch, Martin, 2005, DOI
   `10.1016/j.orl.2004.04.002`.
5. Gleixner et al., PaPILO, 2023, DOI `10.1287/ijoc.2022.0171`.
6. Achterberg, conflict analysis, 2007, DOI
   `10.1016/j.disopt.2006.10.006`.
7. Mexi et al., cut-based conflict analysis, 2025, DOI
   `10.1287/ijoc.2024.0999`.
8. Turner et al., adaptive cut selection, 2023, DOI `10.5802/ojmo.25`.
9. Berthold, Mexi, Salvagnin, Feasibility Jump, 2023, DOI
   `10.1007/s12532-023-00234-8`.
10. van Doornmalen and Hojny, symmetry handling, 2024, DOI
    `10.1007/s10107-024-02102-2`.
11. Applegate et al., PDLP, 2026, DOI `10.1007/s12532-026-00309-2`.
12. Gleixner et al., MIPLIB 2017, 2021, DOI
    `10.1007/s12532-020-00194-3`.
13. Gasse et al., NeurIPS 2019:
    <https://proceedings.neurips.cc/paper/2019/hash/d14c2267d848abeb81fd590f371d39bd-Abstract.html>.
14. Paulus et al., ICML 2022:
    <https://proceedings.mlr.press/v162/paulus22a.html>.

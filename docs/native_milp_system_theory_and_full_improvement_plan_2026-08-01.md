# Native MILP system theory, external progress, and full improvement plan

Date: 2026-08-01  
Last verified: 2026-08-02
Status: architecture and truthfulness remediation in progress; the post-P13
controlled experiment does not establish a Native efficiency gain
Scope: linear MILP, the native branch-and-cut tree, its LP kernels, and the
strict HiGHS/SCIP reference paths

## 1. Executive conclusion

The final post-P13 repeated experiment shows **no evidence that Native MILP
is faster than HiGHS or SCIP**, either generally or on the fixed four-instance
cohort. Native and HiGHS both receive a 30.000-second shifted PAR-10; SCIP has
the best shifted PAR-10, 12.197 seconds, on this selected cohort. Mean
fixed-horizon PDI is 1.984103 seconds for Native, 1.676066 for HiGHS, and
1.211046 for SCIP. All 36 trajectories are available and all 94 primal events pass
original-space audit, but four independent instances are not enough for a
population-level ranking. The D07 repair caps nested root-LNS solves by the
outer remaining budget: current Native calls return in 2.700--2.900 seconds,
whereas the earlier three
`neos-3083819-nubu` calls used 3.91--4.89 seconds under the same 3-second
backend limit. This is deadline-contract evidence, not a solve-rate gain.

P20 now has a real incumbent-driven sequential-tree restart, disabled by
default. The fixed four-instance default-threshold run triggered zero times in
12 attempts, so it adds no efficiency evidence. A deliberately selected
`bell5` forced-trigger stress restarted 3/3 times and reduced median wall time
from 2230.6 ms to 620.6 ms, but one preselected instance is only mechanism
evidence and cannot support default enablement or a general claim.

No local node-count reduction, one-instance proof, unit-test pass, or strict
HiGHS solve may be reported as a Native efficiency improvement. A new claim
requires the fixed repeated Native/HiGHS/SCIP protocol in Section 10 after the
remediation set is frozen.

The main bottleneck is not the absence of one more branching score. It is the
whole chain:

1. benchmark truth and hard deadlines;
2. original-space correctness and postsolve contracts;
3. presolve/domain parity;
4. root and node LP reliability;
5. warm node-LP throughput;
6. cut and primal-heuristic portfolio control;
7. search, restart, and parallel coordination.

The immutable code audit contains 46 formal findings. The 51 normalized audit
rows currently have 25 closed, 22 partial, and 4 open outcomes. With 16
post-audit discoveries, the complete ledger has 41 closed, 22 partial, and 4
open records. These counts are a truthfulness ledger, not a performance score: no
structural performance row is closed without repeated fixed-cohort evidence.

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

The official latest release checked on 2026-08-02 is HiGHS 1.15.1, published
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

### 4.3 Release-source verification and scope

The 2026-08-02 source check used official project release metadata, not search
snippets or third-party summaries:

| Project | Current official release | Relevant verified change | Scope for Native |
|---|---|---|---|
| HiGHS | 1.15.1, 2026-07-02 | First prototype parallel MIP worker/node-search, scheduler/TSAN fixes, retained presolve bounds, basis setters, implied-bound fixes | Primary upstream implementation and differential oracle |
| SCIP | 10.0.3, 2026-07-06 | 10.0 exact rational MILP, cut-based conflicts, implied integrality, reflection symmetry; 10.0.3 correctness/concurrent-mode fixes | Independent correctness and feature oracle |
| PaPILO | 3.0.1, 2026-07-06 | Probing binary-column index correction and RoundingSAT interface update | Presolve/postsolve differential target before upgrading |
| OR-Tools | 9.15, 2026-01-12 | CP-SAT shared-tree/clause work and LRAT proof production/checking; MathOpt solution hints to HiGHS | Architectural/proof reference only; CP-SAT is not a linear-MILP performance oracle |

Version recency is not evidence that a mechanism is useful here. Upgrades must
be isolated beside the local forks, replay the same models/options, and pass
original-space objective/status audits before any code is rebased.

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
| Presolve/domain | PaPILO MIP presolve, event propagation, clique/implication code exist. Native presolve activities are initialized once and updated by column deltas; probe worlds use sparse trails and incident-row wakeups; learned implications are remapped and imported into the production graph. `BCDomain` restores both activity and the saved propagation queue. | Integrated repeated presolve, domain, implication, clique and symmetry feedback. | The four-instance strict-HiGHS oracle has 12/12 valid postsolve incumbents, but this is not full-MIPLIB parity. Several optional reductions remain default-off pending complete proofs, and the probing ablation shows no end-to-end gain. P3. |
| Root LP | HiGHS LP backend is usable; native dual/IPM paths exist. | Mature dual simplex/IPM/crossover. | Native presolved roots and degenerate large LPs remain fragile; IPM is not a reliable default. P4. |
| Node LP reoptimization | Each dispatcher owns a persistent HiGHS node-LP instance; mutable ownership is not stored in queued bases. Dense basis-inverse materialization is removed and inverse rows/BTRAN are lazy. | Factor/basis repair and highly optimized FTRAN/BTRAN/repricing. | Fixed-cohort iteration/reinversion savings are not yet measured; local-row changes still rebuild structure. |
| Cuts | GMI, covers, cliques, implied bounds, transformed cuts and experimental CGLP paths exist. | Integrated selection, numerical safeguards, pool/LP aging and broad separator portfolio. | Family presence exceeds evidence. CGLP stays off; dead disabled MIR implementation was removed. P5. |
| Primal heuristics | Several rounding/repair/local mechanisms and subsolve counters exist. | Feasibility Jump, RENS/RINS, randomized/shifting, sub-MIP scheduling. | No portfolio-level primal-integral ablation. P5. |
| Branching | Reliability probing, directional pseudocosts, exact-result reuse and calibration telemetry exist. Serial and shared parallel reliability paths materialize one active standard form per probing batch, apply each direction through an exact rollback transaction, and use persistent resolves after establishing a separate mutable probe owner. P13 Phase 1 root domain probing now uses one persistent `BCDomainProbeWorkspace`, trail-based sparse extraction, incremental row/clique propagation, and verified rollback; the final cohort records 3 workspaces, 612 worlds/rollbacks, 2,163 changed/exported columns, 12,696 processed rows, 48 implications, and zero failures. Phase 2 root LP probing separately uses one lazy `lb/ub` workspace and one synchronized SF; it records 9 workspaces, 336 transactions/rollbacks, zero failures, 12 cold solves, and 324 persistent resolves. Node fractional frontiers are cached across unchanged LP states. | HiGHS reliability branching and SCIP hybrid strategies are mature. | Strong probing still establishes a separate active form/owner and lacks controlled LP-iteration/reinversion attribution. P13 is structurally closed over both audited root-probing paths, but the post-remediation end-to-end cohort shows no Native speed gain. |
| Queue/restart | Domain dedup uses compact hash buckets with exact bound comparison; hashes never decide equality. The sequential tree has a proof-gated incumbent restart transaction, disabled by default. | Joint queue/dive/restart system with global domain/cut feedback. | Multiple ordered containers, dense active child copies, queue scans, parallel ownership, and naturally triggered restart evidence remain. |
| Conflict/clique/symmetry | Conflict propagation has threshold-indexed variable-to-clause wakeups; unrequested reason construction is skipped. Clique insertions update persistent sorted endpoint adjacency instead of rebuilding both CSR graphs. | HiGHS integrated clique/implication/symmetry; SCIP 10 cut conflicts/reflections and clause management. | LBD/activity aging, lazy persistent reason handles, symmetry reduction, and fixed-cohort payoff remain unestablished. |
| Parallelism | A generation-counted condition variable replaces the 1 ms/5 ms monitor polling path. | HiGHS 1.15 prototype parallel MIP; SCIP concurrent/parallel modes. | Queue locks/scans and deterministic/TSAN/scaling acceptance remain open. |
| Exact certificates | Some proof artifacts and audits exist. | SCIP 10 exact rational mode. | No end-to-end independently checkable Native optimality certificate. P1, then conditional exact mode. |
| Observability | Native, HiGHS, and SCIP expose timestamped primal/dual events to the benchmark. Every primal event retains an original-space incumbent and must pass audit before fixed-horizon PDI is computed. Terminal-certificate filters remove and count transient dual values; availability flags distinguish unknown from zero. The wrapper records hashes/build/host/solver provenance, rotates block order, enforces a POSIX process watchdog, and computes instance-clustered paired PAR-10/PDI intervals. | Solver-native event streams and established benchmark harnesses. | The current cohort has only four independent instances, so its intervals cannot support a general ranking claim. P0. |

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
  clique size from integer zero to `optional<int>`;
- strengthened the field census to distinguish any member access from
  production option use and explicit production statistic mutation, then
  deleted eight more exported-but-never-written counters instead of continuing
  to publish their default zeros;
- replaced all Native-scope direct environment reads with one immutable
  solve-entry snapshot, inherited by nested workers and recorded in
  `BCResult::effective_environment`;
- removed an inert root-oracle control path whose `MIPSOLVERS_*` writes did
  not match the vendored HiGHS `HACDCPF_*` reads; root-oracle controls are now
  explicit per-`Highs` instance options;
- removed a second, more serious false-identity path: the Native tree's
  `bc_try_vendored_highs_root_certificate()` launched a complete HiGHS MIP
  solve without a deadline, checked whether it used at most one node only
  after completion, and could publish that result as Native. The helper, both
  call sites, six statistics fields, and its log fields are gone;
- made each node-LP dispatcher own its persistent HiGHS state, removed eager
  dense basis inversion, and required exact old/new bounds for incremental SF
  updates;
- made native-presolve row activities persistent across reductions, with exact
  finite/infinite contribution deltas for bound changes and fixed columns;
- replaced native-presolve probing's dense bound-world copies with a sparse
  bound/activity trail, incident-row wakeups, deterministic row/implication
  budgets, reverse rollback, surviving-world publication, reduced-index
  remapping, and production implication-graph import;
- repaired native presolve's loss of ranged-row lower bounds, added complete
  integer slope/intercept/divisibility guards to singleton/doubleton
  substitutions, corrected signed parallel-row scaling, and deleted a known
  non-equivalent coefficient-strengthening transform rather than leaving it
  reachable behind an opt-in flag;
- made `BCDomain` rollback restore cached activities and the saved pending-row
  queue, and removed the path that returned success with propagation work left;
- replaced whole-graph clique CSR reconstruction on every insertion with
  persistent sorted endpoint adjacency and exact duplicate insertion;
- bounded strong-branch probes to 50-500 simplex iterations, disabled their
  expensive fallback on limit, stopped temporary heuristic/probe results from
  retaining an SF copy, cached unchanged node fractionality, removed the third
  repeated root down-propagation, and disabled eager reduced-cost proof resolve
  by default;
- replaced parallel monitor polling, all-clause conflict fixed-point scans,
  eager unrequested reason construction, and per-node binary domain-signature
  strings with event/indexed or compact exact-checked paths;
- made the MIPLIB runner isolate every solver invocation on POSIX, terminate it
  after `limit + grace`, and distinguish backend soft limits from hard process
  timeouts;
- made unavailable MIPLIB counters and diagnostics JSON `null` and CSV empty,
  including strict-HiGHS branching/fallback fields that previously looked like
  observed zeros;
- repaired a deferred-node lifetime bug in which a cleared `x_relax` could be
  queued without `lp_refresh_needed` and later indexed as an integer LP
  solution. Missing primals now force refresh, unsuccessful refresh remains
  deferred, and original-space feasibility checks reject malformed vectors.
- cleared stale `row_lhs` state after native presolve reconstructs lower sides
  as explicit negated upper rows, and froze a pre-search reduced model for
  final incumbent validation so cutoff-only rows cannot reject their source
  incumbent;
- normalized both direct-HiGHS maximization LP objectives into Native's
  internal minimization convention before they can become node bounds;
- changed objective-cutoff artifact accounting from source-level scalar sums
  to per-target attribution, so a direct objective term and an implied event
  cannot charge the same target contribution twice;
- added a 128-model exhaustive binary oracle differential that alternates
  objective sense and PaPILO, covers upper/lower/ranged/equality rows, and
  explicitly retains the D15 case-112 optimum regression.

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

Current finite differential evidence, 2026-08-02: 128 deterministic six-binary
models are enumerated over all 64 assignments and compared to the full Native
production path with tree-exhaustion certification. This found and now guards
the direct-HiGHS maximization-sign defect (D14) and objective-artifact
per-target double count (D15). Passing 128/128 closes those concrete defects;
it does not satisfy the broader P1 acceptance gate or establish general
correctness.

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

Current P20 status, 2026-08-02: the sequential tree now implements the restart
transaction portion of this phase. The controller requires a proof-valid
incumbent, a nonempty frontier, configurable node/improvement/time gates, and a
restart budget. It discards the old frontier, requeues a clean root under the
current root domain, clears node-local payloads, rebuilds the active standard
form, and preserves solve-global cuts, conflicts, implications, clique edges,
and pseudocost observations. This is `PARTIAL`: naturally triggered
heterogeneous evidence, parallel ownership, and the acceptance gate above
remain outstanding.

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

## 10. Numerical evidence

Sections 10.1 and 10.2 predate the current audit remediation set. Their raw
files remain useful for reproducing the defects that motivated the audit, but
their timings, solve counts, and rankings must not be cited as measurements of
the current working tree. Section 10.3 is the post-P16 intermediate run;
Section 10.15 is the current post-P13 evidence.

**D10 seed-provenance correction.** Every report in Sections 10.1--10.6 stored
`seed=0`, but the benchmark did not assign that value to Native `BCOptions`.
HiGHS and SCIP received zero; Native actually used its deterministic default
`0x9E3779B97F4A7C15`. In addition, a directly supplied Native zero seed selected
`random_device` only in the work-stealing pool while remaining deterministic in
the heuristic RNG. The code now makes every seed, including zero,
deterministic and passes the benchmark seed to Native. Consequently, every
"seed zero" phrase in the historical sections below means the recorded
benchmark setting, not the effective Native seed, and none of those reports is
post-D10 fixed-seed evidence.

### 10.1 Reliability-branching experiment

The reliability-branching experiment used a fixed cohort of `50v-10`,
`enlight_hard`, `neos-3083819-nubu`, and `wachplan`, one thread, recorded seed zero,
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

### 10.2 Historical truthfulness fix and controlled 3-second experiment

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
named solver paths, one thread, recorded seed zero, a 3-second backend limit, a 2-second
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

The historical conclusion was that SCIP won this selected cohort and that the
experimental native LP kernel was noncompetitive. The one-instance Native B&C
plus HiGHS-LP result is not admissible as current evidence after the subsequent
correctness, ownership, termination, and configuration changes.

### 10.3 Historical post-P16 controlled experiment

The post-P16 run used the fixed four-instance cohort, Native B&C with
HiGHS LP, HiGHS MIP 1.14.0, and SCIP MIP 9.0.0; three repetitions; one thread;
recorded seed zero; a 3-second backend limit; a 2-second process-watchdog grace; and a
`1e-4` gap. Solver order was rotated within every instance-repeat block, so
each solver occupied each order position exactly four times. The immutable
manifest records executable and instance SHA-256 hashes, the solution file,
commit and dirty source-state hashes, compiler/build flags, host, complete
command, and solver versions. Reports are:

- `reports/miplib2017_audit_p16_2026-08-02.json`;
- `reports/miplib2017_audit_p16_2026-08-02.csv`.

The earlier `miplib2017_audit_controlled_2026-08-02` report is the immediate
pre-P16 baseline and remains immutable for comparison.

| Solver path | Runs | Audited incumbents | Proven | Hard timeouts | Shifted PAR-10 |
|---|---:|---:|---:|---:|---:|
| Native B&C + HiGHS LP | 12 | 6 | 0 | 0 | 30.000 s |
| HiGHS MIP | 12 | 9 | 0 | 0 | 30.000 s |
| SCIP MIP | 12 | 9 | 3 | 0 | 12.144 s |

All 36 isolated invocations returned without a signal or hard timeout, and
every published incumbent passed the original-space audit. Native found no
incumbent on `enlight_hard` or `wachplan`. Every Native call returned within
the 5-second process watchdog.
The removed hidden HiGHS-MIP path and the deferred empty-primal crash are not
present in this run. A separate current-binary stress report repeats Native on
`enlight_hard` ten times: all ten runs return normally at the soft limit with
576--581 nodes and no incumbent, signal, or hard timeout. Its files are
`reports/miplib2017_native_enlight_stability_2026-08-02.json` and the matching
CSV. This is stability evidence, not solve-performance evidence.

The paired Native/SCIP PAR-10 ratio is 11.2518. Bootstrap resampling is clustered
by the four independent instances, not by the 12 correlated repeat rows; its
95% percentile interval is `[1.0, 1424.518]`. That interval is intentionally
wide and includes 1.0. PDI is explicitly `null` because no bound-event series
is available. Therefore the admissible conclusion is narrow: SCIP is best on
this selected cohort; Native and HiGHS both receive the PAR-10 penalty on all
rows; the repairs restore identity, deadline containment, and stability, but
do not establish a Native efficiency improvement or a population-level solver
ranking. P16 does establish that compensated activity arithmetic retains a
unit contribution in the constructive `1e16 + 1 - 1e16` row and that presolve
no longer eliminates a high-impact near-fixed continuous column. Those are
numerical-correctness results; the unchanged feasible/proven counts and Native
PAR-10 mean there is still no measured end-to-end efficiency gain.

### 10.4 Post-P02 presolve audit, ablation, and current default run

Four reports separate presolve correctness, P02 mechanism payoff, and current
default product behavior:

- `reports/miplib2017_presolve_equivalence_p02_corrected_2026-08-02.json`
  runs Native presolve, solves the reduced model through the strict HiGHS MIP
  contract, applies Native postsolve, and audits the original MPS;
- `reports/miplib2017_audit_p02_corrected_2026-08-02.json` disables PaPILO so
  the rewritten Native probing path is exercised in the Native tree;
- `reports/miplib2017_audit_p02_no_probing_2026-08-02.json` uses the same
  Native settings but disables only Native presolve probing;
- `reports/miplib2017_audit_post_p02_2026-08-02.json` uses the default Native
  configuration and the fixed Native/HiGHS/SCIP comparison protocol.

Every report was produced by `tools/run_miplib2017_audit.py`. Each JSON embeds
the executable, instance, and solution-file SHA-256 hashes; the exact command;
commit and dirty source-state hashes; compiler/build/host data; solver
versions; and an instance-clustered paired PAR-10 analysis where at least two
solvers are present. The earlier
`reports/miplib2017_audit_p02_2026-08-02.json` is intentionally not replaced.
It records one hard timeout and two invalid `wachplan` postsolve results. Those
failures exposed D03: Native presolve had discarded every `LPModel::row_lhs`
value before any probing logic ran.

After preserving ranged-row lower bounds, the strict presolve oracle produced
12/12 feasible original-space-audited incumbents across three repeats, proved
6/12 runs within a 20-second limit, had zero hard timeouts, and had maximum row
violation `1.70e-11`. This establishes the tested reductions' postsolve parity
on the four-instance corpus. It does not establish full-MIPLIB correctness.

The P02 implementation no longer copies complete `lb/ub` vectors. A probe
world pushes only changed bounds, updates only incident row activities, wakes
only incident rows, and rolls all changes back in reverse order. Conditional
implications are retained across rollback, remapped after deleted columns, and
imported into the production binary implication graph. One-sided conflicts fix
the trigger and publish the surviving world's bounds. Deterministic row-visit
and implication budgets plus a one-second deadline cap the work.

| Instance | Trail pushes | Row visits | Learned/imported implications | Max touched columns | Truncated | Presolve on/off |
|---|---:|---:|---:|---:|:---:|---:|
| `50v-10` | 4,000 | 5,000 | 1,000 / 1,000 | 3 | no | median 1.40 / 0.273 ms |
| `enlight_hard` | 1,458 | 2,880 | 500 / 500 | 6 | no | median 0.547 / 0.090 ms |
| `neos-3083819-nubu` | 0 | 0 | 0 / 0 | 0 | no | median 3.18 / 3.04 ms |
| `wachplan` | 8,620 | 100,000 | 7,097 / 7,097 | 261 | yes | median 104.04 / 5.87 ms |

The on/off Native runs have identical outcome aggregates: 3/12 audited
incumbents, 0/12 proofs, zero hard timeouts, and a 30-second shifted PAR-10.
On `wachplan`, probing reduces 10 additional columns but consumes the complete
100,000-row budget; the tree processes 5 nodes and 52 LP solves with probing,
versus 1 node and 30 LP solves without it, and neither run finds an incumbent.
This is no evidence of an efficiency gain. P02 closes because the audited dense
copy/discard defect is gone and the learned structure reaches production, not
because the cohort became faster.

The post-P02 default comparison is:

| Solver path | Runs | Audited incumbents | Proven | Hard timeouts | Shifted PAR-10 |
|---|---:|---:|---:|---:|---:|
| Native B&C + HiGHS LP | 12 | 6 | 0 | 0 | 30.000 s |
| HiGHS MIP 1.14.0 | 12 | 9 | 0 | 0 | 30.000 s |
| SCIP MIP 9.0.0 | 12 | 9 | 3 | 0 | 12.145 s |

Every published incumbent passes the original-space audit. Each solver occupies
each block-order position four times. The paired Native/SCIP PAR-10 ratio is
10.8455 with an instance-cluster bootstrap 95% interval of
`[1.0, 1275.696]`; PDI remains `null` because the runner has no bound-event
stream. Native's `neos-3083819-nubu` calls use 3.91--4.89 seconds despite a
3-second backend limit, but remain below the 5-second hard watchdog. The honest
conclusion is therefore unchanged: no current experiment demonstrates a
Native efficiency improvement, and SCIP is best only on this small selected
cohort. Root/node LP throughput, complete deadline compliance, broader
presolve parity, and trajectory observability remain higher-priority work than
additional branching-score tuning.

### 10.5 Post-D07 global-deadline repair and historical default run

The post-P02 trace exposed a separate correctness-of-measurement defect. Root
LNS assigned every nested MILP `opt.lns_time_limit` and stopped the stage only
after `lns_time_limit * lns_max_iters`. It never capped a child by the outer
remaining time or the root-finalization reserve. On `neos-3083819-nubu`, root
LNS alone used about 3.495 seconds and drove a nominal 3-second Native call to
3.910--4.886 seconds.

D07 adds a pure budget allocator and applies it before every LNS child. The
child budget is

$$
  \min\{T_{\mathrm{call}},\ T_{\mathrm{stage}}-t_{\mathrm{stage}},\
         T_{\mathrm{outer}}-t_{\mathrm{outer}}-T_{\mathrm{finalize}}\},
$$

and no child starts when this value is below the minimum useful budget. The
allowance is recomputed after the restricted model is constructed, so model
construction is charged to the outer solve. A deterministic regression covers
the 3-second outer-limit case and exhausted-reserve cases.

The targeted report
`reports/miplib2017_audit_deadline_d07_neos_2026-08-02.json` repeats Native
three times. Calls end in 2.880--2.891 seconds, all three incumbents pass the
original-space audit, and each retains objective `6316801`. The corresponding
pre-repair calls used 3.910--4.886 seconds with that same incumbent. Thus the
repair removes 1.02--2.01 seconds of deadline overrun; it does not improve the
incumbent, close the gap, or prove the instance.

The historical post-D07 full report is
`reports/miplib2017_audit_post_d07_2026-08-02.json`, with raw rows in the
matching CSV. It repeats the same four instances, three solvers, three runs,
one thread, recorded seed zero, rotated block order, 3-second backend limit, 2-second
hard-watchdog grace, and `1e-4` gap:

| Solver path | Runs | Audited incumbents | Proven | Hard timeouts | Shifted PAR-10 |
|---|---:|---:|---:|---:|---:|
| Native B&C + HiGHS LP | 12 | 6 | 0 | 0 | 30.000 s |
| HiGHS MIP 1.14.0 | 12 | 9 | 0 | 0 | 30.000 s |
| SCIP MIP 9.0.0 | 12 | 9 | 3 | 0 | 12.144 s |

Every published incumbent passes the original-space audit, no invocation hits
the hard watchdog, and all Native calls return in 2.700--2.895 seconds. The
Native/SCIP paired PAR-10 ratio is 11.2474 with an instance-cluster bootstrap
95% interval of `[1.0, 1422.826]`. PDI remains unavailable because the runner
does not expose bound events. The admissible conclusion is exact: D07 restores
the tested global-deadline contract and changes neither Native's 6/12 feasible,
0/12 proven outcome nor its 30-second PAR-10. There is still no measured Native
efficiency gain.

### 10.6 Historical post-D09 certified progress trajectories

The benchmark now records structured bound events from Native, HiGHS, and
SCIP. A primal event is usable only when it carries a complete incumbent that
passes the original MPS row, bound, integrality, and objective audit. The PDI
horizon is the configured backend limit. The normalized gap is piecewise
constant and equals one until both a primal and dual bound exist.

D09 was exposed by this stricter contract. Native temporarily reported a queue
bound of 20.5 on `enlight_hard` and later retained only 20.0 after LP recovery
failed. SCIP can likewise emit a solving-stage dual callback during
`SCIPfree()` that is stronger than its returned final certificate. Neither
transient value is admissible trajectory evidence. The producer/benchmark now
remove such values only after the solver lifecycle is complete, publish the
discard count, and reject any remaining primal or dual regression. No monotone
envelope is fabricated.

The historical pre-D10 reports are
`reports/miplib2017_audit_progress_events_post_d09_2026-08-02.json` and the
matching CSV. The protocol is the same four instances, three repeats, one
thread, recorded seed zero, rotated solver order, 3-second backend limit, 2-second hard
watchdog grace, and `1e-4` relative gap:

| Solver path | Runs | Audited final incumbents | Proven | PDI rows | Mean PDI | Shifted PAR-10 |
|---|---:|---:|---:|---:|---:|---:|
| Native B&C + HiGHS LP | 12 | 6 | 0 | 12 | 1.989 s | 30.000 s |
| HiGHS MIP 1.14.0 | 12 | 9 | 0 | 12 | 1.676 s | 30.000 s |
| SCIP MIP 9.0.0 | 12 | 9 | 3 | 12 | 1.209 s | 12.144 s |

All 36 event streams are available, all 90 primal events pass original-space
audit, no call hits the hard watchdog, and three SCIP exit events are explicitly
counted as discarded terminally uncertified dual values. Native/HiGHS paired
PDI ratio is 1.109 with instance-cluster bootstrap 95% interval
`[0.873, 1.630]`. Native/SCIP is 6.284 with interval `[0.934, 249.966]`.
Native/SCIP paired PAR-10 ratio is 11.1477 with interval `[1.0, 1385.330]`.

These measurements add trajectory evidence but no efficiency gain. Native
still finds incumbents in only 6/12 calls and proves none; HiGHS finds 9/12 and
proves none; SCIP finds 9/12 and proves three. The Native/HiGHS PDI interval
includes one, and every population-level interval is dominated by the four-
instance sample. E08 therefore remains open.

### 10.7 P06/P14 storage mechanisms and post-D10 reproducibility failure

P06 now compacts queued serial, parallel, and recursive sub-MIP domains to
sparse changes relative to the root domain. Nine proof/cut payload vectors use
copy-on-write ownership. A serial or parallel split creates one dense child
domain by copy and transfers the materialized parent domain to the other child
by move. Recursive sub-MIP splitting now uses the same one-copy/one-move
ownership pattern instead of copying both siblings. One $O(n)$ dense-domain
copy still remains per materialized split, and compact domains are still
materialized on queue pop. P06 is therefore `PARTIAL`, not a completed
path-delta tree and not a speed result.

P14 stores separator candidates sparsely for integer-rounding, cover,
basis/row MIR, projected capacity, transformed-tableau/path, clique, and
zero-half families. One row-major base snapshot plus a sparse committed-row
overlay replaces per-family full matrix snapshots. The overlay is populated
from rows actually inserted in `lp.A`, rather than from pre-validation
candidates, so later families see the exact same-round LP rows. Focused cover
and row-MIR coefficient/validity tests execute 33 assertions.

The post-D10 one-instance mechanism report is
`reports/miplib2017_p14_seed_contract_smoke_2026-08-02.json`. It runs Native
B&C with the HiGHS LP kernel on `enlight_hard`, one thread, effective seed zero,
a one-second backend limit, a two-second hard-watchdog grace, and five
repetitions. Every run records exactly 469 sparse candidates, 10,940 candidate
entries, a peak of 81 candidates/3,644 entries, and 1,355 temporary dense
mathematical workspaces containing 398,140 values. This establishes that the
sparse candidate path is active and that dense transformed-cut work remains.
It does not measure a speedup.

More seriously, the five nominally identical runs explored 192, 192, 200, 200,
and 334 nodes; added 122, 122, 164, 164, and 116 cuts; and used 815, 815, 832,
832, and 1,015 LP solves. One run reached the soft time limit, none found an
incumbent, and none hit the hard watchdog. Since the seed, thread count,
separator-storage counters, executable, and instance were fixed, seed wiring
alone does not make the Native path reproducible. Time-driven root heuristics,
LP numerical recovery, or another state-ordering source changes the tree.
Until that divergence is isolated, this report is negative evidence for P22
and E08. At this historical point P14 remained `PARTIAL` because full-matrix
cut insertion and dense mathematical workspaces remained. Section 10.12 records
the later insertion repair and its still-negative end-to-end experiment.

### 10.8 Post-D10/P14 fixed-cohort experiment

The first cohort with the repaired seed contract is
`reports/miplib2017_audit_post_d10_p14_2026-08-02.json`, with raw rows in the
matching CSV. It uses the same four instances and three solvers, three repeats,
one thread, effective seed zero, balanced block order, a three-second backend
limit, a two-second hard-watchdog grace, and a `1e-4` gap. The manifest records
the executable and input hashes, dirty source-state hash, exact command,
compiler/build/host data, solver versions, and empty effective environment.

| Solver path | Runs | Audited incumbents | Proven | Mean PDI | Hard timeouts | Shifted PAR-10 |
|---|---:|---:|---:|---:|---:|---:|
| Native B&C + HiGHS LP | 12 | 6 | 0 | 1.988 s | 0 | 30.000 s |
| HiGHS MIP 1.14.0 | 12 | 9 | 0 | 1.770 s | 0 | 30.000 s |
| SCIP MIP 9.0.0 | 12 | 9 | 3 | 1.209 s | 0 | 12.144 s |

All 36 bound-event streams are present. All 92 primal events carry a complete
original-space incumbent and pass row, bound, integrality, and objective
audits. There are zero failed final audits, zero event-stream errors, and zero
hard timeouts. Three SCIP exit dual events not retained by its final
certificate are counted and discarded. Thirty-three calls end at their soft
limit; only SCIP's three `enlight_hard` calls prove optimality.

Native/HiGHS has paired fixed-horizon PDI ratio 1.055 with an
instance-clustered 95% interval of `[0.809, 1.544]`. Their PAR-10 ratio is one
because neither proves any cohort row. Native/SCIP has PAR-10 ratio 11.144 with
interval `[1.0, 1384.114]` and PDI ratio 6.257 with interval
`[0.922, 249.565]`. Four independent instances make these population
intervals intentionally wide.

The result is not an efficiency improvement. Native remains at 6/12 feasible
and 0/12 proven, has a worse mean PDI than HiGHS, and provides no measurable
P14 wall-time benefit. SCIP is best on this selected cohort, while the broad
intervals forbid a population ranking. E06 closes because the repaired
benchmark protocol is now exercised end to end; E08 remains open.

### 10.9 P20 incumbent-driven tree restart

The implementation is exercised at three evidence levels. All reports use one
thread, seed zero, three repeats, a three-second backend limit, and a two-second
hard-watchdog grace.

First, `reports/miplib2017_p20_restart_off_2026-08-02.json` and
`reports/miplib2017_p20_restart_on_default_2026-08-02.json` use the same four
instances as Section 10.8. The enabled policy permits two restarts and requires
256 nodes since the preceding restart, 64 open nodes, a 1% relative incumbent
improvement, and 0.25 seconds remaining. It triggered 0/12 times. Both groups
have 6/12 audited incumbents, 0/12 proofs, and 30.000-second shifted PAR-10.
Mean fixed-horizon PDI is 1.9896 seconds off and 1.9939 seconds on. Because the
mechanism never ran, that difference is noise rather than a restart effect.

Second, `reports/miplib2017_p20_restart_forced_stress_2026-08-02.json` allows
one restart with zero node, improvement, and remaining-time thresholds and one
required open node. It still triggered 0/12 times: no run in this short cohort
combined a newly published tree incumbent with a nonempty frontier. This
report demonstrates that permissive options alone do not manufacture a
restart event.

Third, the preselected triggering instance `bell5` is reported in
`reports/miplib3_bell5_p20_restart_off_2026-08-02.json` and
`reports/miplib3_bell5_p20_restart_forced_2026-08-02.json`. All three forced
runs restart once at node 306, discard 171 open nodes, and preserve 8 cut-pool
rows, 53 conflicts, 261 implications, 32 clique edges, and 708 pseudocost
observations. Median solve time falls from 2230.6 ms to 620.6 ms and the node
count from 3861 to 1458. This is a repeatable 3.59x result on one deliberately
selected mechanism stress, not a population estimate.

Decision: keep restart disabled by default and P20 `PARTIAL`. The positive
single-instance result warrants a broader trigger-rich cohort; the zero-trigger
formal cohort prevents any default or general efficiency claim. E08 remains
open.

### 10.10 Exhaustive small-MILP correctness differential

The fixed-seed test generates 128 feasible MILPs with six binary variables and
five inequality rows. Row types cycle through upper-only, lower-only, and
ranged constraints; every third model also has an equality. Objective sense
alternates, PaPILO is toggled on alternating cases, and the exact oracle
enumerates all $2^6=64$ assignments. Native must report tree-exhausted optimal,
match the oracle objective to $10^{-7}$, and return an integral source-feasible
solution.

Before repair, maximization cases exposed the direct-HiGHS sense-boundary bug
D14. After D14, case 112 exposed D15: scalar artifact accounting counted the
same target-$x_3$ contribution once through the direct objective candidate and
once through the event candidate for literal $x_4=1$. It derived the invalid
conflict $x_3=1\Rightarrow x_4=0$ and returned $-5$ although enumeration gives
$-6$ at $(0,1,1,1,1,1)$. Per-target maximum/union accounting removes that
conflict while preserving additive contributions from distinct target columns.

After both repairs the differential passes 128/128 models and 1,410 assertions.
The full MILP executable passes 54 test cases and 2,029 assertions, and CTest
passes 20/20 registered tests. These counts are correctness evidence for a
small finite class. They do not measure speed, replace MIPLIB differentials,
or justify closing E08.

### 10.11 Post-D15 fixed-cohort experiment

The post-D15 baseline report is
`reports/miplib2017_audit_post_d15_2026-08-02.json`, with raw rows in the
matching CSV. It repeats the four instances from Section 10.8 three times with
Native B&C/HiGHS LP, HiGHS MIP 1.14.0, and SCIP MIP 9.0.0. Each run uses one
thread, effective seed zero, a three-second backend limit, a two-second hard
watchdog grace, and relative gap $10^{-4}$. The manifest payload SHA-256 is
`f902c8ede2c5fa4ef0a59244e47082e44810246ce324a15b919f697bf47bcf27`.

| Solver | Runs | Audited incumbents | Proven | Mean PDI | Hard timeouts | Shifted PAR-10 |
|---|---:|---:|---:|---:|---:|---:|
| Native B&C + HiGHS LP | 12 | 6 | 0 | 1.985 s | 0 | 30.000 s |
| HiGHS MIP 1.14.0 | 12 | 9 | 0 | 1.670 s | 0 | 30.000 s |
| SCIP MIP 9.0.0 | 12 | 9 | 3 | 1.212 s | 0 | 12.144 s |

All 36 event streams are available and have no stream error. All 90 primal
events pass original-space audit. Four SCIP dual events were removed because
the terminal SCIP certificate did not retain those transient solving-stage
bounds; the report publishes the removal count. Native/HiGHS has paired PDI
ratio 1.104 with instance-cluster bootstrap interval `[0.857, 1.642]`.
Native/SCIP has ratio 6.189 with interval `[0.893, 249.389]`. The intervals are
wide because there are only four independent instances.

Conclusion: D12-D15 improve correctness and auditability, not demonstrated
efficiency. Native remains 0/12 proven and has worse mean PDI than both
references. SCIP remains best on this selected cohort. P20 stays disabled by
default and `PARTIAL`; E08 stays `OPEN`.

### 10.12 P06/P14 final fixed cohort

`add_sparse_rows_to_lp()` previously copied every old nonzero into a triplet
array, added the new cut entries, rebuilt the complete matrix with
`setFromTriplets()`, and compressed it after every separator batch. The live
path now validates sparse rows, extends the row dimension, reserves per-column
capacity, and appends entries in monotonically increasing row order with
Eigen's `insertBackUncompressed()`. The zero-caller dense-row rebuild helper was
deleted. Direct HiGHS serialization already creates a compressed boundary copy
when required. The remaining direct `outerIndexPtr()` consumers either use
`nonZeros()` or explicitly compress their row-major diagnostic copy.

The focused repeated-append regression executes 154 assertions. It checks the
exact old and new coefficients, RHS values, ranged-row lower bounds, rejection
of malformed/nonfinite/empty rows, repeated non-compressed appends, and fewer
storage reallocations than append calls in a controlled same-support sequence.
The production result exposes six counters rather than inferring the mechanism
from wall time: append calls, appended rows, appended entries, prior entries
that bypass triplet reconstruction, storage reallocations, and peak spare
entry slots.

The final frozen report is
`reports/miplib2017_audit_p06_p14_final_2026-08-02.json`, with raw
rows in the matching CSV. It uses the same four-instance, three-solver,
three-repeat, one-thread, seed-zero, three-second protocol as Section 10.11.
Its manifest payload SHA-256 is
`201dc50283904b29636e36df397ff3f0e136170e2b9157a2e929761906f6ef73`.

| Solver | Runs | Audited incumbents | Proven | Mean PDI | Hard timeouts | Shifted PAR-10 |
|---|---:|---:|---:|---:|---:|---:|
| Native B&C + HiGHS LP | 12 | 6 | 0 | 1.986850 s | 0 | 30.000 s |
| HiGHS MIP 1.14.0 | 12 | 9 | 0 | 1.671126 s | 0 | 30.000 s |
| SCIP MIP 9.0.0 | 12 | 9 | 3 | 1.212551 s | 0 | 12.144 s |

All 36 event streams are present, 91/91 primal events pass original-space
audit, four transient SCIP dual events are explicitly discarded, and no call
hits the hard watchdog. Native's 12 runs execute 36 matrix appends containing
465 rows and 34,149 new entries. They bypass triplet reconstruction of 131,361
prior entries, but still trigger 33 storage reallocations; the maximum reported
spare capacity is 5,076 entries. The memory/time tradeoff is therefore visible
rather than hidden.

The same Native runs record 1,830 dense branch-domain copies, 1,075,632 copied
bound values, and 1,830 moves. They record zero sub-MIP compactions. Thus the
main-tree one-copy/one-move transfer is active, but this cohort does not execute
the recursive sub-MIP queue mechanism and cannot establish its performance.

Against HiGHS, the final paired PDI ratio is 1.107 with an
instance-clustered 95% interval `[0.866, 1.640]`; the interval crosses one.
Native/SCIP PDI ratio is 6.274 with interval `[0.897, 258.724]`, and its
PAR-10 ratio is 11.278 with interval `[1.0, 1434.414]`. P06 and P14 remain
`PARTIAL`: one dense child-domain copy per split, Eigen capacity growth, and
dense transformed-cut workspaces remain, and the selected cohort shows no
aggregate efficiency gain. E08 remains `OPEN`.

### 10.13 Production-path re-audit of P03, P05, and P11

P03 has a real persistent-state implementation in both sequential and parallel
tree dispatchers. Each dispatcher owns the mutable persistent HiGHS LP, while
queued node basis snapshots detach the mutable owner. This is meaningful
architecture work, but local structural changes still rebuild and the final
cohort has no LP-reinversion attribution. P03 stays `PARTIAL`.

P05 has one Native production call site for
`update_standard_form_bounds_incremental()`, used to commit a root-probe
one-sided infeasibility fixing after its directional transaction rolls back.
Strong-branch directions use `StandardFormBoundTransaction`. Native MILP still
contains 33 full `update_standard_form_bounds()` call sites: 31 in `branch_and_cut.cpp`, one in
`bc_branching.cpp`, and one in `bc_parallel.cpp`, spanning node, heuristic,
repair, dive, sub-MIP, audit, and probe paths. The incremental API is therefore
live but not systemic. P05 stays `PARTIAL`.

P11 no longer copies a complete standard form per direction. Both the serial
current-node path and the shared reliability helper used by parallel search
materialize one active form per probing batch, transact each direction, and
roll back only touched bound/RHS/objective scalars. In the final P13 report, 27
base materializations serve 101 strong directions, with 101 rollbacks, zero
transaction failures, 88 persistent resolves, and 13 cold solves. This is real
production coverage, but it is not a controlled before/after attribution and a
separate active form/owner must still be established. P11 stays `PARTIAL`.

### 10.14 D16 cross-node propagation-reason repair

An independently feasible SCIP objective-37 witness for `enlight_hard` exposed
an unsound Native conflict. Parent row propagation derived a bound and a
`DomainReasonBound`, but the normal path discarded the reason before child
conflict analysis. The child then resolved an incomplete frontier into global
unary no-goods, including `x101 >= 2 infeasible`, `x105 <= 1 infeasible`, and
`x167 <= 0 infeasible`, and falsely exhausted the queue after 198 nodes.

Row and proof propagation reasons now live as long as their derived node bounds.
Optional source-tagged audit identified `conflict_pool` as the witness-violating
source; the normal path allocates no event vector. Three default-round witness
runs reached the time limit at 530/539/537 nodes without invalid publication or
witness violation, and a 10-round stress reached 868 nodes rather than false
exhaustion. This closes D16 only. Native still found no `enlight_hard`
incumbent, and the post-D16 four-instance report showed no efficiency gain.

### 10.15 P13 root domain and LP probing transactions

The original P13 finding is primarily about Phase 1 root domain probing. The
old path copied complete `lb/ub` vectors for each down/up world and then scanned
all columns to export implications. Removing the redundant third down world did
not close that finding, and the earlier claim that the separate Phase 2 root LP
transaction alone closed P13 was incorrect.

Phase 1 now owns one persistent `BCDomainProbeWorkspace`. It saves the domain
trail, applies a single-variable fixing, incrementally propagates incident rows
and cliques from newly fixed literals, collects only unique trailed columns,
exports sparse bound deltas, and restores and validates only those columns.
One-sided infeasibility fixings are committed through the same persistent
domain. The final cohort records 3 workspaces, 612 worlds and rollbacks, 2,163
trail pushes, 2,163 unique changed/exported columns, 12,696 processed rows, 48
learned implications, no committed fixing, and zero failures. The removed
unconditional extraction loop would have visited 122,400 columns on those same
`612 x 200` worlds. This is structural-work evidence, not wall-time evidence.

Phase 2 separately retains one lazy `lb/ub` workspace, one synchronized
standard form, exact `StandardFormBoundTransaction` rollback, and a separate
mutable LP owner. Nine Native runs record 9 workspaces, 336 transactions and
rollbacks, zero failures, 12 cold solves, and 324 persistent resolves.

The frozen working-tree report is
`reports/miplib2017_audit_p13_root_domain_probe_transactions_2026-08-02.{json,csv}`.
It uses four instances, three solvers, three repeats, one thread, seed zero, a
3-second backend limit, a 2-second watchdog grace, and relative gap `1e-4`.

| Solver | Runs | Audited incumbents | Proven | Mean PDI | Hard timeouts | Shifted PAR-10 |
|---|---:|---:|---:|---:|---:|---:|
| Native B&C + HiGHS LP | 12 | 6 | 0 | 1.984103 s | 0 | 30.000 s |
| HiGHS MIP 1.14.0 | 12 | 9 | 0 | 1.676066 s | 0 | 30.000 s |
| SCIP MIP 9.0.0 | 12 | 9 | 3 | 1.211046 s | 0 | 12.197 s |

All 36 streams are available, all 94 primal events pass audit, three transient
SCIP dual events are discarded explicitly, and no process hits the watchdog.
Native/HiGHS has paired PDI ratio 1.102021 with 95% interval
`[0.868196, 1.586477]`; Native/SCIP has PDI ratio 4.807893 with interval
`[0.909243, 114.777614]` and PAR-10 ratio 8.610117.

The manifest payload SHA-256 is
`c89122b537ae5900dbefcffae9f3833e7edb78eb9e76127aac49dd45e2ab1789`;
the JSON SHA-256 is
`c244bdc3e22a89180b9c53e973df9526dbf760103f1bb5954fc89a25dbc3a74e`;
the CSV SHA-256 is
`c9e7500bfafbb88aa2595719e69268246a745b2ada754f2ff372fb0ade49a27a`.
The manifest records a dirty, fully hashed source state
`95819a55b2bdf3897ebfa3fb6f3da5004363959dab32a3e372c16bbadf929d54`.

P13 is structurally closed over both audited root-probing paths. It does not
close the performance question: Native remains 0/12 proven, its shifted PAR-10
is 30.000 seconds, and the fixed cohort provides no end-to-end efficiency
improvement evidence. E08 remains `OPEN`; P11 and P19 remain `PARTIAL`.

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
15. PaPILO v3.0.1 official release, 2026-07-06:
    <https://github.com/scipopt/papilo/releases/tag/v3.0.1>.
16. OR-Tools v9.15 official release, 2026-01-12:
    <https://github.com/google/or-tools/releases/tag/v9.15>.

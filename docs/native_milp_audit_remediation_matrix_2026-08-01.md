# Native MILP audit remediation matrix

Date: 2026-08-01  
Last verified: 2026-08-02
Baseline: `docs/native_milp_math_model_and_audit.tex`  
Rule: the audit document is immutable. This file records remediation against
the current working tree; it does not rewrite or weaken an audit finding.

## Status contract

- `CLOSED`: the deceptive/correctness path is removed or repaired and has a
  focused test or direct census evidence.
- `PARTIAL`: a real part is repaired, but the audit's complete performance or
  reproducibility contract is not met.
- `OPEN`: the audited structural problem is still present.
- A passing unit test does not establish general MILP correctness or speed.
- No performance row is closed by code inspection alone. It also needs a fixed,
  repeated numerical cohort.

## Current count

The immutable audit contains 46 formal findings: 15 subsystem findings, 23
severity-ranked summary findings (5 severity-1, 8 severity-2, 6 severity-3,
4 severity-4), and 8 placeholder/false-complexity findings. Some summary
findings deliberately restate subsystem root causes, so 46 is not a count of
independent patches.

This remediation matrix normalizes those findings into 51 acceptance rows.
Against the current working tree, 25 are `CLOSED`, 22 are `PARTIAL`, and 4 are
`OPEN`. Performance rows remain `PARTIAL` until the repeated fixed-cohort
experiment is complete, even when their audited data-structure defect has been
removed.

Sixteen additional defects were discovered while executing this matrix. They are
not retroactively counted among the audit's 46 findings or the 51 normalized
rows. Across the 67 normalized and post-audit records, 41 are `CLOSED`, 22 are
`PARTIAL`, and 4 are `OPEN`:

| ID | Post-audit discovery | Status | Evidence |
|---|---|---|---|
| D01 | A path described as a vendored HiGHS root certificate actually launched a complete HiGHS MIP solve without a deadline, checked `nodes <= 1` only after completion, and returned the result under the Native solver identity | CLOSED | `bc_try_vendored_highs_root_certificate`, its two call sites, six statistics fields, and `rootCert` logging were deleted. In the controlled rerun, all 12 Native invocations returned without a hard timeout |
| D02 | A deferred child could be queued after its primal vector was cleared, without `lp_refresh_needed`; the later integer-feasibility check indexed the empty vector and intermittently crashed | CLOSED | Deferred or missing-primal nodes are forced through LP refresh, unsuccessful refresh cannot clear the flag, and `satisfies_with_bounds()` validates dimensions and finiteness. The controlled rerun completed 36/36 isolated calls without a signal or hard timeout; a separate 10-repeat `enlight_hard` stress run also completed without either failure |
| D03 | Native presolve initialized every `A` row with lower bound `-inf` and discarded `LPModel::row_lhs`; all ranged/lower-bound rows could therefore disappear or be weakened before postsolve | CLOSED | `init_from_lp()` preserves each supplied `row_lhs`. The ranged-row regression reconstructs `x0+x1>=1` as `-x0-x1<=-1`; the three-repeat presolve-oracle report has 12/12 original-space-audited incumbents and maximum row violation `1.70e-11` |
| D04 | Integer singleton/doubleton substitutions did not enforce all divisibility conditions; the doubleton path checked an integer slope but not an integer intercept | CLOSED | Integer singleton substitution now requires integral scaled RHS and coefficients on remaining integer variables. Integer doubleton substitution requires both integral slope and intercept. Focused half-integral RHS regressions reject the invalid substitutions |
| D05 | Parallel-row merging divided row bounds by the coefficient ratio although the coefficients were mapped by multiplication | CLOSED | Bounds are multiplied by the signed ratio, with lower/upper sides swapped for a negative ratio. The focused `2x+2y<=3` versus `x+y<=2` regression retains the correct tighter row |
| D06 | An opt-in binary coefficient-strengthening transform changed the feasible set and therefore could publish an invalid reduced model | CLOSED | The option, statistic, implementation, run branch, and model-change condition were deleted; the Native tree has zero remaining symbol hits for this unsupported reduction |
| D07 | Root LNS gave every nested MILP its uncapped local time limit and treated `per-call limit * iterations` as a substitute for the outer global deadline | CLOSED | Every LNS child is now capped by the per-call limit, remaining LNS-stage allowance, and `outer remaining - finalization reserve`, recomputed immediately before the child solve. The budget helper regression covers a 3-second outer limit. On `neos-3083819-nubu`, three audited calls fell from 3.910--4.886 s before the repair to 2.880--2.891 s after it, with the same audited incumbent objective and no hard timeout |
| D08 | `bc_types.hpp` contained an unused alternate `TreeNode`/LCA domain manager/arena that claimed sparse path-delta tree management while the production tree used the dense `Node` below it | CLOSED | The complete zero-caller alternate stack and its private `BoundChange` type were deleted. The production type is now named honestly. P06 remains `PARTIAL`: queued domains and payloads are compact, but `branch_child()` still performs an `O(n)` dense-domain copy |
| D09 | Progress trajectories published transient dual bounds that were not retained by the solver's terminal certificate; Native could regress from a queue bound after LP recovery failure, and SCIP could emit a stronger solving-stage callback while leaving `SCIPfree()` | CLOSED | Native filters queue bounds against its retained final certificate after search. The benchmark performs the same terminal-certificate filter after each external solver lifecycle, counts every discarded event, and rejects any remaining primal/dual regression. A three-repeat `enlight_hard` smoke discarded two Native events per run; the post-D10 36-run report discarded three SCIP exit events. All 36 streams remained available and all 92 primal events passed original-space audit |
| D10 | Benchmark JSON reported `seed=0`, but the Native path never copied that seed into `BCOptions`; separately, zero meant `random_device` only in the work-stealing pool despite being deterministic in Native heuristics | CLOSED | The benchmark now assigns `options.random_seed=cfg.seed`. Every seed value, including zero, is deterministic throughout Native. A two-pool regression reproduces the exact zero-seed victim sequence. Reports produced before D10 remain historical and do not establish the claimed effective-zero-seed Native protocol |
| D11 | Objective clique partitions and implication-event capacity aggregation could count the same mandatory objective contribution twice. On the complete-graph `K8` minimum vertex-cover model, the valid LP lower bound 4 and clique lower bound 7 were incorrectly promoted to 8; objective-cutoff extraction then emitted eight invalid unary conflicts, fixed every binary to one, and falsely declared objective 8 optimal although the true optimum is 7 | CLOSED | Partition-owned target columns are excluded from aggregate/capacity accounting and cutoff artifact extraction while ordinary implication activation remains live. The focused production regression solves `K8` with objective propagation enabled, requires objective 7, and audits the returned original-space solution. The full MILP test suite passes |
| D12 | Native presolve's `rebuild_model()` reconstructed every surviving two-sided row as one or two upper-bound rows but left the source `LPModel::row_lhs` vector attached. Its stale entries could then be interpreted as lower bounds of unrelated rebuilt rows | CLOSED | `rebuild_model()` now clears `row_lhs` after materializing every finite lower side as an explicit negated upper row. The ranged-row regression preserves `x0+x1>=1`, rebuilds it as `-x0-x1<=-1`, and requires an empty stale-lower-bound vector |
| D13 | Final reduced-space incumbent validation used the live `base_lp`, which may contain objective-cutoff search rows that are valid only for strict improvement and intentionally exclude the incumbent that created the cutoff | CLOSED | The pre-cut reduced source model is frozen as `pre_cut_root_lp`; final reduced-space feasibility is audited against that model, followed by postsolve and original-space validation. Search-only cutoff rows remain available for pruning but cannot reject the incumbent they were derived from |
| D14 | Both direct-HiGHS LP completion paths stored HiGHS' user-direction maximization objective `+z` as a Native internal minimization bound, although every Native maximization bound is represented as `-z`. This allowed wrong pruning and wrong optimality claims | CLOSED | Both LP-kernel boundaries now normalize by objective sense. A deterministic exhaustive-oracle suite alternates minimization/maximization and fails on multiple maximization cases without the repair; all 128 cases now agree with exact enumeration |
| D15 | Objective-cutoff artifact extraction summed a direct objective contribution and an implication-event contribution even when both represented the same target column. In deterministic case 112 this manufactured `2+2>3.00001`, published the false implication `x3=1 => x4=0`, removed the true optimum `x=(0,1,1,1,1,1)`, and declared `-5` optimal instead of `-6` | CLOSED | Each literal candidate now retains `target column -> contribution`; duplicate sources merge by per-target maximum, while contributions to distinct targets still add. Unary and pair conflicts use the target-union value. The case-112 regression explicitly requires oracle/result `-6`, and the full 128-case production differential passes |
| D16 | Row and proof propagation could derive a bound at a parent node but discard its `DomainReasonBound` before the child was analyzed. A later child conflict then resolved against an incomplete local frontier and published the result globally. On `enlight_hard`, this produced false unary no-goods such as `x101 >= 2 infeasible`, `x105 <= 1 infeasible`, and `x167 <= 0 infeasible`; the conflict pool excluded SCIP's objective-37 witness at depth 4 and Native falsely exhausted the queue after 198 nodes | CLOSED | Node-domain closure now persists row and proof reasons for the same lifetime as their derived bounds. Optional source-tagged propagation audit identified `conflict_pool` as the witness-violating source without allocating event vectors on the normal path. The parent/child regression requires conflict analysis to resolve through the inherited reason to the real branch literal, and the zero-round regression enforces the documented row-round budget. After repair, three default-round witness-audited runs reached the time limit at 530/539/537 nodes with no invalid conflict publication or witness violation; a 10-round stress reached 868 nodes and the time limit instead of the false 198-node exhaustion. This closes this reason-lifetime defect only; it is not a general correctness or speed proof |

## A. Correctness and truthfulness

| ID | Audit finding | Status | Current evidence | Acceptance evidence still required |
|---|---|---|---|---|
| A01 | Direct `solve_milp_bc` skipped integrality validation; free general integers entered the tree | CLOSED | Direct entry validates conflicting declarations and rejects two-sided unbounded integer variables in `legacy/branch_and_cut.cpp`; status constants are in `bc_status.hpp` | `test_milp_solver`: direct-declaration and free-integer cases |
| A02 | Native presolve silently lost infeasibility | CLOSED | `PresolveStats::infeasible`, `mark_infeasible()`, empty-row/activity/probing checks, and top-level early return are wired | `test_presolve`: infeasible empty row and both probing sides infeasible |
| A03 | A perturbed LP optimum could certify the original node | CLOSED | L1 perturbation can provide a basis only; the original LP must be resolved before success is publishable | `test_branch_and_cut`: perturbed fallback cannot certify original node |
| A04 | Quantized hashes silently merged distinct cuts, literals, reasons, clauses, implications, or node domains | CLOSED | Hashes are prefilters only; exact comparators back every repaired path. Node signatures encode exact bound bytes after a hash prefilter | Collision regression in `test_milp_solver` covers clauses, reasons, cuts, shared pools, and node domains |
| A05 | `SharedIncumbent` published the objective before the matching solution | CLOSED | `try_update()` now rechecks under `x_mtx`, installs `x`, then release-publishes objective and availability | Shared-incumbent publication regression plus parallel solver tests |
| A06 | `StrictHiGHS` actually ran the native tree with a HiGHS LP oracle | CLOSED | `make_strict_highs_production_options()` sets `strict_highs_mip_contract=true`; the strict adapter runs the HiGHS MIP contract | Strict adapter contract regression in `test_milp_solver` |
| A07 | Maximization warm-start early return exposed the internal minimization sign | CLOSED | Public objective is recomputed in original sense/space before publication | Warm-start objective equals `c.dot(x)` regression |
| A08 | Reduced-cost fixing could use the wrong active bound side and publish false progress | CLOSED | Fixing now requires a matching active lower/upper status and proof-compatible result | Reduced-cost regression in `test_branch_and_cut` |
| A09 | Default ignored `gap_tol` and required near-exhaustive tree proof | CLOSED | `require_tree_exhaustion_certificate=false` by default; exact exhaustion remains opt-in | Default/strict-option regressions and benchmark termination classification |
| A10 | Reachable FlowCover family had an acknowledged validity gap | CLOSED | `FlowCover` was removed from the public enum, implementation, dispatch, and statistics | Build plus no-symbol census |
| A11 | Row-based ImpliedBound cut merely reproduced its source row | CLOSED | The duplicate separator and public cut choice were removed. Variable-bound-table and objective implication machinery remain because they derive different artifacts | Build plus no-separator-symbol census |
| A12 | Model-row cover used a 512-bin approximate lifting mechanism | CLOSED | Approximate sequential lifting was deleted; the separator now emits the unlifted valid cover. Exact transformed-tableau lifting remains a separate implementation | Solver regression suite; future exact lifting needs its own validity tests |
| A13 | Public callback and warm-start APIs advertised unsupported state and then threw | CLOSED | Node/cut selectors and restart payload types were deleted. `BCWarmStart` contains only validated primal hints | Extended API regression in `test_milp_solver` |
| A14 | `StrategyDispatcher::analyze()` returned fabricated zeros; `estimate_best_solver()` ignored them | CLOSED | Both unused pseudo-analysis APIs were deleted. Dispatcher documentation now states its real policy/availability contract | Compile-time no-symbol census |
| A15 | Dead `search/parallel/root` stack and orphan headers were compiled and advertised conflicting implementations | CLOSED | Sources and headers are deleted; CMake no longer compiles them | CMake build and path census |
| A16 | Nine public options and nineteen statistics fields had zero uses | CLOSED | Original dead fields were deleted. A strengthened census finds no zero member-access or zero production-use fields among 222 options, and no zero member-access or zero explicit-production-mutation fields among 424 stats fields. Eight additional exported-but-never-written statistics discovered by the strengthened check were deleted; later telemetry fields, including strong-branch, root-domain-probe, and root-LP-probe counters, are production-mutated and benchmark-exported | Preserve the census command with every public-field change |
| A17 | `implied_events_by_literal` was built but ignored by propagation | CLOSED | Each round now gathers candidate events from active literal buckets with stamp deduplication; malformed/missing index falls back to a full scan. Capacity and one-missing proofs still scan all events | Chained literal fixed-point and fallback regressions in `test_branch_and_cut` |
| A18 | Cut-derived domain loops computed floor/ceil values and only incremented counters | CLOSED | Non-applying loops and the phantom statistic were deleted. Remaining cut-row code populates the variable-bound table/implication graph, which is consumed | No `variable_bound_cut_domain_tightenings` member; implication consumers remain live |
| A19 | Cut-pool hash and aging were decorative; eviction was FIFO-like | PARTIAL | Dedup uses a support-hash bucket followed by real support/parallelism/RHS comparison. New cuts start at age 0. Serial/shared selection ages and purges transactionally | Full violation selection still scans the pool; fixed-cohort time/memory evidence is pending |
| A20 | `incumbent_quality_reject_factor=1e6` was effectively an empty quality gate | PARTIAL | The gate is live at all incumbent publication paths and catches extreme repair artifacts; zero/non-finite disables it | Its default threshold remains heuristic and must not be marketed as a correctness proof |
| A21 | Root-oracle controls appeared configurable but used incompatible environment prefixes | CLOSED | Native wrote `MIPSOLVERS_*` while vendored HiGHS read `HACDCPF_*`, so the controls were inert. They are now explicit per-`Highs` instance options and the process-global environment shim is deleted | Build and root-oracle regressions; no process-global oracle state remains |

## B. Structural performance findings

| ID | Audit finding | Status | Current evidence | Work required to close |
|---|---|---|---|---|
| P01 | Presolve repeatedly recomputes row activities | PARTIAL | Row activities are initialized once. Bound changes and fixed-column removal update only incident rows by exact finite/infinite contribution deltas; structural edits recompute only the affected row. Probing also updates only incident activities and rolls them back from its sparse bound trail. `PresolveStats` and focused regressions expose recomputation/delta work | There is no frozen pre-change activity-work trace. The fixed cohort shows correctness but no end-to-end gain, so the performance claim remains unclosed |
| P02 | Presolve probing copies full bound vectors and discards learned structure | CLOSED | Each probe world now records only changed bounds, wakes only incident rows, applies chained propagation, and restores bounds/activities in reverse trail order. Conditional implications survive the world rollback, are remapped after column deletion, and are imported into the production `BinaryImplicationGraph`; a one-sided conflict fixes the trigger and publishes the surviving-world bounds. Row-visit/implication budgets and the one-second deadline set an explicit cap | Sparse-touch, transitive implication, one-sided conflict/postsolve, reduced-index remap, budget rollback, and production-import regressions pass. In the three-repeat cohort, probing learned/imported 1,000 implications on `50v-10`, 500 on `enlight_hard`, and 7,097 on `wachplan`; the last case stopped exactly at 100,000 row visits. The on/off ablation had identical 3/12 feasible, 0/12 proven, and 30 s PAR-10 results, so closure is structural/correctness evidence, not a speed claim |
| P03 | Persistent node LP exists but normal tree paths clear `cached_sparse_basis` | PARTIAL | Each sequential/parallel `SolverDispatcher` enables persistent LP state and owns one mutable persistent HiGHS LP for `NodeLP`; queued basis snapshots detach the mutable owner, so siblings share only immutable basis/status state. A production-call re-audit confirms both tree dispatchers use this path | Fixed-cohort node-LP iteration, reinversion, and wall-time evidence; local-cut structural changes still require rebuilds |
| P04 | Vendored wrapper materializes a dense basis inverse | CLOSED | Eager `m x m` inverse materialization was deleted. `BasisOps` supplies inverse rows, BTRAN, and tableau rows lazily | `test_dual_simplex` asserts an empty dense inverse and a valid finite lazy inverse row |
| P05 | Known bound changes still call full `update_standard_form_bounds`; incremental API has zero callers | PARTIAL | The API requires exact old/new values, rejects stale state transactionally, and handles infinite/finite transitions. Root LP probing uses it to commit a one-sided infeasibility fixing after the directional transaction is rolled back; strong-branch directions use the stricter rollback transaction interface. All propagation deltas carry old/new values. A production-call re-audit finds 33 full `update_standard_form_bounds()` call sites in Native MILP: 31 in `branch_and_cut.cpp`, one in `bc_branching.cpp`, and one in `bc_parallel.cpp` | Arbitrary node switching and heuristic/dive/sub-MIP paths still perform full-domain scans. Replace only after exact state-transition ownership is available and add call/work telemetry |
| P06 | Every open node stores dense `lb/ub` and deep-copied proof/cut payloads | PARTIAL | Serial, parallel, and recursive sub-MIP queues compact inactive node domains to sparse deltas from the root domain and materialize them on pop. Nine branch proof/cut payload vectors use copy-on-write storage. Serial and parallel splits use one dense child-domain copy plus one move; recursive sub-MIP splitting now uses the same ownership pattern instead of copying both siblings. Production telemetry reports queue storage plus dense copies, values copied, and moves | One `O(n)` dense-domain copy remains per materialized split, and compact queued domains are materialized on pop. The final cohort records 1,830 copies, 1,075,632 copied bound values, and 1,830 moves. It records zero sub-MIP compactions, so it does not exercise or support a speed claim for the recursive change. Replace materialized branching with true path-delta transitions and obtain memory/wall-time evidence before closing |
| P07 | Conflict propagation has no watched literals or variable-to-clause index | PARTIAL | ConflictPool now indexes lower/upper bound occurrences by variable and threshold. Propagation seeds only satisfied occurrences/unary clauses and wakes affected clauses after each tightening; reason vectors are built only when requested | Add activity/LBD aging and compare indexed wakeups on a fixed conflict-heavy cohort |
| P08 | Reason tracking performs eager quadratic materialization and unbounded arena growth | PARTIAL | If neither reasons nor a learned conflict are requested, reason arrays and row-wise reason merges are skipped entirely. Equality-row ID buffers are reused. Telemetry regression requires zero materialized reason clauses on the untracked path | Persistent lazy reason handles and bounded/epoch arena reclamation remain |
| P09 | Node queue uses multiple ordered containers plus vector scans/string signatures | PARTIAL | Per-node binary domain strings were replaced by compact hash buckets followed by exact bound comparison, so collisions cannot decide equality | Replace the remaining multiset/DFS-vector combination and queue-wide incumbent scans |
| P10 | Parallel workers scan queues under global locks; monitor polls every 1 ms | PARTIAL | A generation-counted condition variable now wakes the monitor on node progress/worker transitions. Deadline-aware 50 ms maximum waits replace 1 ms polling, and generation-stable double checks replace the 5 ms sleep | Queue-wide operations and global-lock contention remain; measure scaling before closing |
| P11 | Strong branching copies full standard forms and cold-solves probes | PARTIAL | Local exact evidence, directional reliability, reuse signatures, and duplicate-sample prevention are implemented. Serial current-node probing and the shared reliability helper used by parallel search each materialize one active probe standard form for a probing batch. Every direction applies its exact bound delta through `StandardFormBoundTransaction`, solves, and rolls back only the touched bound/RHS/objective scalars; it no longer copies a standard form per direction. In the final P13 cohort, 27 base materializations serve 101 direction transactions, with 101 rollbacks, zero failures, 88 persistent resolves, and 13 cold solves | This is a structural copy reduction, not an end-to-end speed result. A node-level active form and a separate mutable LP owner still have to be established, and the current cohort shows no aggregate Native advantage. Obtain a controlled pre/post attribution for LP iterations, reinversions, PDI, and wall time, or safely transact on a cloned/current-node active owner, before closing |
| P12 | Fractionality is rescanned across all integer variables at multiple sites | PARTIAL | A node caches its fractional branchable frontier after the final LP/domain state. Queue estimates and unchanged-LP branching reuse it in both serial and parallel paths; every LP/domain refresh explicitly invalidates it | Add scan/cache-hit measurements and replace remaining heuristic-specific full scans where they are shown material |
| P13 | Root probing repeats a fixed branch and performs full-vector extraction | CLOSED | The earlier removal of the redundant third down world did not close the audit finding: Phase 1 root domain probing still copied complete `lb/ub` vectors for every down/up world and scanned all columns to export implications. It now owns one `BCDomainProbeWorkspace`, initializes row activities once, applies row and clique propagation from the changed trail, exports only unique trailed columns, validates rollback against each column's recorded old bounds, and commits one-sided infeasibility fixings through the same persistent domain. The state machine was extracted from the monolithic solve body and has a direct row-to-clique cascade/rollback/commit regression. Phase 2 root LP probing separately uses one lazy bound workspace, one synchronized standard form, exact transactions, and a separate mutable probe owner. In the final cohort, Phase 1 records 3 workspaces, 612 worlds/rollbacks, 2,163 trail pushes, 2,163 changed/exported columns, 12,696 processed rows, 48 learned implications, and zero failures; the old unconditional export loop would have visited 122,400 columns on those same worlds. Phase 2 records 9 workspaces, 336 transactions/rollbacks, zero failures, 12 cold solves, and 324 persistent resolves | The fixed cohort still shows no end-to-end Native gain: 6/12 feasible, 0/12 proven, 30.000 s PAR-10, and 1.984103 s mean PDI. Closure is for both audited repeated-domain-probe work and the separate root-LP copy mechanism, not a speed claim |
| P14 | Separators allocate dense `n` vectors and rebuild row-major workspaces | PARTIAL | Candidate pools are sparse and `add_cuts()` owns one row-major snapshot plus a committed-row overlay. `add_sparse_rows_to_lp()` no longer copies all old entries into triplets: it validates rows, extends the row dimension, reserves per-column capacity, and appends in increasing row order with `insertBackUncompressed`. The zero-caller dense rebuild interface was deleted. HiGHS serialization compresses only at its boundary; all direct compressed-storage assumptions in Native MILP were audited. A 154-assertion repeated-append regression preserves old coefficients/RHS/`row_lhs`, rejects malformed rows, and exercises amortized capacity. Production telemetry reports calls, rows, new entries, prior entries bypassing triplet reconstruction, reallocations, and peak spare slots | This is a real structural change, not a demonstrated solver speedup. The final P06/P14 cohort records 36 appends, 465 rows, 34,149 new entries, and 131,361 prior entries bypassing triplet reconstruction, but still 33 storage reallocations and 5,076 peak spare slots. Transformed mathematics still creates dense workspaces. Native remains 0/12 proven and its mean PDI is 1.986850 s versus 1.671126 s for HiGHS. Broaden the cohort and replace repeated Eigen capacity growth or batch all family rows before closing |
| P15 | Clique-table updates rebuild CSR graphs | PARTIAL | The table now owns persistent sorted per-variable/per-literal adjacency vectors. Incremental insertion binary-searches and touches only edge endpoints; there is no CSR unpack/rebuild. A regression checks exact duplicate handling, variable/literal consistency, and storage stability for unrelated adjacency | Compare insertion/query time and memory on a fixed clique-heavy cohort before closing |
| P16 | Propagation uses cancellation-prone residual arithmetic and absolute tolerances | PARTIAL | `StableActivitySum` uses Neumaier accumulation plus FMA product residuals in presolve, `BCDomain`, ordinary/tracked node propagation, and reason-tracked domain propagation. Residuals exclude the target contribution stably; proof-audit activities use the same replacement operation; bound comparisons are ULP-aware; near-fixed continuous columns are no longer eliminated by an absolute gap test. The unused duplicate 629-line legacy presolve was deleted. Constructive `1e16 + 1 - 1e16` regressions cover all live propagation paths and prevent a false presolve infeasibility | Node propagators still rebuild compensated activities when a dirty row is visited rather than maintaining one event-updated cache, and the fixed `1e-15` coefficient cutoff is not row-scaled. The controlled rerun preserves correctness but shows no aggregate performance gain, so this performance row cannot close |
| P17 | Dual proof construction is eager after incumbents | PARTIAL | `enable_reduced_cost_proof_cut_resolve` now defaults to false and has a default-contract regression, so normal nodes do not eagerly build reduced-cost proof rows. Explicit opt-in remains available for proof experiments | Measure proof-path benefit/cost and replace explicit eager mode with an actual conflict/cutoff request interface before closing |
| P18 | LP data is copied through LPModel, standard form, HiGHS arrays, and result forms | PARTIAL | `SimplexOptions::retain_standard_form` is false for Probing, FeasPump and LPDive, with dispatcher tests requiring those temporary results to have an empty form while NodeLP retains it | Input SFs and HiGHS serialization still copy; persistent model ownership and view-based result/proof interfaces remain |
| P19 | `BCDomain::restore()` re-marks every row, losing backtrack incrementality | PARTIAL | Savepoints capture the trail and pending-row queue. Restore reverses activity deltas before clearing failed-probe state, restores only the saved queue, and no longer re-marks every row. Propagation cannot return success with rows still queued. Native presolve and root domain probing now follow the sparse trail/rollback discipline. Root production telemetry reports 612 worlds and matching rollbacks, 2,163 trail pushes/unique changed columns, 12,696 processed rows, and zero rollback/closure failures | The fixed cohort exercises this root path only on `enlight_hard`; add a broader conflict-heavy paired cohort and tree-domain repair telemetry before claiming a general payoff |
| P20 | No incumbent-driven tree restart | PARTIAL | The sequential production tree has an incumbent-driven restart transaction, disabled by default. A guarded controller checks restart count, nodes since restart, open frontier, relative incumbent improvement, proof-valid incumbent state, and remaining time. A restart discards every queued node, requeues a clean root under the current root domain, rebuilds the node standard form, clears all node-local payloads, and preserves solve-global cuts, conflicts, implications, clique edges, and pseudocost observations. Unit, queue-transaction, and production-trigger regressions pass; worker JSON/CSV expose trigger and preservation counters | The fixed four-instance, three-repeat default-threshold experiment triggered 0/12 times, so it supplies no efficiency evidence. A forced-threshold `bell5` mechanism stress triggered 3/3 times and reduced the median from 2230.6 ms/3861 nodes to 620.6 ms/1458 nodes, but it is one preselected triggering instance and cannot justify a general claim or default enablement. Freeze a broader heterogeneous cohort with naturally triggered runs, compare proof-valid outcomes/PDI/PAR-10, and tune gates without instance-name rules |
| P21 | Early concurrent tree is hard-disabled; cut worker is normally gated off | OPEN | `allow_concurrent_tree=false`; cut worker requires tree cuts | Remove only after root-state ownership is safe and experiments justify it |
| P22 | Parallel search is not fully deterministic | PARTIAL | Turn token and consistent incumbent snapshots reduce scheduling variance; LP completion timing and floating reduction order still vary | Deterministic event ordering or explicitly documented best-effort preset |

## C. Engineering and reproducibility

| ID | Audit finding | Status | Current evidence | Work required to close |
|---|---|---|---|---|
| E01 | Entire live engine is one approximately 35k-line function | OPEN | `branch_and_cut.cpp` is still 37,826 lines. The 190-line root-domain probing state machine is now a separate component, but the principal solve function remains monolithic | Continue extracting state-owning phases only with focused contracts; this first extraction does not close E01 |
| E02 | `BCOptions` is an approximately 230-field flat public blob | OPEN | Dead fields are gone, but 222 extracted fields remain and internal tuning is still public | Separate stable public policy, experimental options, and internal autotuning |
| E03 | Hidden environment configuration prevents replay | CLOSED | Native MILP scope has zero direct `getenv` calls. Solve entry captures one immutable `MIPSOLVERS_*`/`HACDCPF_*` snapshot, nested workers bind it, and `BCResult::effective_environment` records the sorted non-empty settings | Snapshot immutability, worker inheritance, subsequent-solve recapture, and result-provenance regressions |
| E04 | Documentation and names contradicted actual behavior | PARTIAL | StrictHiGHS, public LP-backend comment, presolve comment, warm-start docs, and false RINS claim are corrected. A production-comment re-audit removed unqualified instance-specific timings, `faster`/`fastest` assertions, and `speedup` multipliers from the Native MILP surface without changing behavior. The executable census now fails if those claim forms return | Audit historical performance documents outside the Native MILP production surface; retain only claims tied to a named immutable report and protocol |
| E05 | Phase/P-number comments imitate a roadmap inside production APIs | CLOSED | Native scope has zero P-number roadmap labels and zero `TODO/FIXME/HACK/XXX/placeholder/scaffold/not implemented` hits. Remaining `Phase 1/2/3` comments describe currently executed algorithm phases rather than future work | Preserve the scoped census and keep future work in design documents |
| E06 | Benchmark outcomes could be mislabeled or compare inconsistent objectives | CLOSED | The runner validates original-space objective/status, isolates every invocation behind a hard watchdog, balances block order, records hashes/build/solver provenance, and reports instance-clustered paired PAR-10/PDI intervals. D10 repaired the false Native seed provenance, and the final P06/P14 cohort has 0 hard timeouts, 0 failed available-incumbent audits, 0 event-stream errors, and 91/91 audited primal events | Preserve the protocol and reject a complete trajectory if any primal event lacks an original-space-audited vector or either bound regresses |
| E07 | Theory documents mixed implemented facts and plans | PARTIAL | System theory now separates Native, vendored HiGHS, and external references and states no general speed win | Recheck every version/literature statement and attach source URLs/commits |
| E08 | No current experiment supports a general Native efficiency claim | OPEN | The final P13 report has complete fixed-horizon PDI, but Native remains 6/12 feasible and 0/12 proven, ties HiGHS at 30.000 s PAR-10, and has mean PDI 1.984103 s versus HiGHS 1.676066 s. The paired Native/HiGHS PDI ratio is 1.102 with 95% interval `[0.868, 1.586]`, so this selected cohort does not distinguish them. SCIP proves 3/12, with 12.197 s PAR-10 and 1.211046 s mean PDI | Freeze a substantially larger heterogeneous cohort and repeat it after another material algorithmic performance change. Four instances and trajectory observability do not support a general ranking |

## Current census and non-claims

Reproduced against the working tree on 2026-08-02:

- `BCOptions`: 222 extracted data fields; zero with no member
  access in `src/include/benchmark/tests` under the audit's method.
- `BCStats`: 424 extracted data fields; zero with no member access and zero
  without an explicit production mutation under the strengthened method. This
  still does not prove that each statistic is meaningful.
- Native MILP environment access: zero direct `getenv` call sites under
  `include/mipsolvers/engine/detail` and `src/engine/solver/native/milp/bc`.
  Reads are routed through the immutable solve snapshot; vendored HiGHS owns
  separate diagnostics outside this census.
- Native placeholder-word census: zero `TODO/FIXME/HACK/XXX/placeholder/
  scaffold/not implemented` hits and zero P-number roadmap labels. Remaining
  `Phase` labels describe currently executed algorithm phases.
- Native production-comment census: zero unqualified `faster`/`fastest`/
  `speedup` or `measured on/to/rationale` claims. Performance evidence belongs
  in a named immutable report with its protocol, not in source comments.
- The main translation unit remains 37,826 lines; root-domain probing now owns a
  separate 190-line implementation and 78-line contract header.
- The census is executable as
  `python3 tools/audit_native_milp_surface.py --repo-root .`; it strips comments
  and literals before counting member accesses.

## Current controlled experiment

The latest default-configuration controlled report is
`reports/miplib2017_audit_p13_root_domain_probe_transactions_2026-08-02.json`
with raw rows in the matching CSV. It uses four fixed instances, Native B&C
with HiGHS LP, HiGHS MIP, and SCIP MIP; three repetitions; one thread;
effective seed zero; a 3-second backend limit; a 2-second process-watchdog
grace; and a `1e-4` relative gap. Each solver occupies each block-order position
exactly four times.

The pre-D10 `miplib2017_audit_progress_events_post_d09_2026-08-02` report is
retained as historical evidence. Its Native path used the deterministic
default seed rather than the recorded zero and must not be cited as satisfying
the repaired seed protocol.

| Solver | Runs | Audited incumbents | Proven | PDI rows | Mean PDI | Hard timeouts | Shifted PAR-10 |
|---|---:|---:|---:|---:|---:|---:|---:|
| Native B&C + HiGHS LP | 12 | 6 | 0 | 12 | 1.984 s | 0 | 30.000 s |
| HiGHS MIP 1.14.0 | 12 | 9 | 0 | 12 | 1.676 s | 0 | 30.000 s |
| SCIP MIP 9.0.0 | 12 | 9 | 3 | 12 | 1.211 s | 0 | 12.197 s |

The Native/SCIP paired PAR-10 ratio is 8.610 on this cohort. The instance-
cluster bootstrap 95% interval is `[1.0, 638.303]`. Fixed-horizon PDI uses a
3-second horizon, piecewise-constant normalized gap, and gap one until both
bounds exist. Native/HiGHS has paired PDI ratio 1.102 with interval
`[0.868, 1.586]`; Native/SCIP has ratio 4.808 with interval
`[0.909, 114.778]`. All 36 streams are available. Every one of 94 primal events
carries a complete original-space vector and passes the audit. Three transient
SCIP exit bounds were discarded because the final solver certificate did not
retain them; the count is published rather than hidden. No call hit the 5-second
hard watchdog, and Native calls returned in 2.700--2.900 seconds. The manifest
payload SHA-256 is `c89122b537ae5900dbefcffae9f3833e7edb78eb9e76127aac49dd45e2ab1789`.
The final JSON SHA-256 is
`c244bdc3e22a89180b9c53e973df9526dbf760103f1bb5954fc89a25dbc3a74e`;
the CSV SHA-256 is
`c9e7500bfafbb88aa2595719e69268246a745b2ada754f2ff372fb0ade49a27a`.
The manifest records a dirty but fully hashed working tree with source-state
SHA-256 `95819a55b2bdf3897ebfa3fb6f3da5004363959dab32a3e372c16bbadf929d54`;
it must not be cited as a clean-commit artifact.
Four independent instances leave the intervals too wide for a general ranking, and
Native shows no aggregate improvement. These limitations keep E08 `OPEN`.

## Exhaustive small-MILP correctness differential

`test_milp_solver` contains a deterministic production-path differential with
128 generated models. Every model has six binary variables, five mixed
upper/lower/ranged rows, an equality row on every third case, alternating
minimization/maximization sense, and alternating PaPILO enablement. The oracle
enumerates all 64 assignments, so it is independent of any LP/MIP backend.
Native is required to exhaust the tree, return the exact oracle objective, and
return an integral solution feasible in the generated source model.

The audit discovered D14, D15, and later D16 rather than merely confirming the
code. With the repairs applied, the differential passes all 128 models and 1,410
assertions. The full `test_milp_solver` run passes 54 cases and 2,029
assertions; the complete CTest registration passes 20/20 tests. This is strong
evidence for this finite generated class, not a proof of general MILP
correctness and not performance evidence.

The P02-specific reports disable PaPILO so Native presolve is exercised:

- `miplib2017_audit_p02_corrected_2026-08-02` runs probing;
- `miplib2017_audit_p02_no_probing_2026-08-02` is the paired Native ablation;
- `miplib2017_presolve_equivalence_p02_corrected_2026-08-02` solves each
  reduced model with strict HiGHS and audits Native postsolve in original space.

The probing on/off runs both have 3/12 audited incumbents, zero proofs, and a
30-second shifted PAR-10. The equivalence oracle has 12/12 audited incumbents,
6/12 proofs, zero hard timeouts, and maximum row violation `1.70e-11`. The
earlier `miplib2017_audit_p02_2026-08-02` report is intentionally retained: it
contains one hard timeout and two invalid `wachplan` postsolve results that led
to D03. It is failure evidence, not a baseline to average away.

## P20 restart experiment

The fixed four-instance restart-off and default-threshold restart-on reports
are `miplib2017_p20_restart_off_2026-08-02` and
`miplib2017_p20_restart_on_default_2026-08-02`. Both use three repeats, one
thread, seed zero, a 3-second backend limit, and a 2-second hard-watchdog grace.
The on policy is the code default: at most two restarts, at least 256 nodes
since the preceding restart, 64 open nodes, 1% relative incumbent improvement,
and 0.25 seconds remaining. It triggered zero times in all 12 runs. Off and on
both produced 6/12 audited incumbents, 0/12 proofs, and 30.000-second shifted
PAR-10. Their mean PDI values, 1.9896 and 1.9939 seconds, cannot be attributed
to a mechanism that never ran.

The same cohort with maximally permissive gates and one allowed restart is
`miplib2017_p20_restart_forced_stress_2026-08-02`. It also triggered zero times
in 12 runs because no run combined a new tree incumbent with a nonempty open
frontier during the horizon. This is a trigger-coverage failure, not neutral
performance evidence.

The deliberately selected `bell5` mechanism stress is frozen as
`miplib3_bell5_p20_restart_off_2026-08-02` and
`miplib3_bell5_p20_restart_forced_2026-08-02`. Every forced run restarted once
at node 306, discarded 171 open nodes, requeued one root, and preserved 8 cut
rows, 53 conflicts, 261 implications, 32 clique edges, and 708 pseudocost
observations. Across three repeats, the median changed from 2230.6 ms and 3861
nodes to 620.6 ms and 1458 nodes. This one preselected triggering instance is
mechanism evidence only. It does not justify the default, a general speed
claim, or closing P20.

The project must not currently claim that Native MILP is generally faster than
HiGHS or SCIP. The latest admissible aggregate conclusion is that SCIP was best
on this small controlled cohort. P16 establishes a constructive numerical
correctness improvement and removes a false-infeasibility mechanism. P02
removes dense probe-world copies and retains implications. P14 removes explicit
triplet reconstruction during sparse cut insertion but still reallocates on
33/36 production appends and shows no aggregate PDI gain. P20 now has one
positive mechanism stress but no triggered fixed-cohort evaluation. None of
these facts supports a general Native efficiency improvement.

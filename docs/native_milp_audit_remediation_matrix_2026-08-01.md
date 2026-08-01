# Native MILP audit remediation matrix

Date: 2026-08-01  
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
| A16 | Nine public options and nineteen statistics fields had zero uses | CLOSED | Original dead fields were deleted. Current repeat census finds no zero member-access fields among approximately 217 options and 354 stats fields | Preserve the census command with every public-field change |
| A17 | `implied_events_by_literal` was built but ignored by propagation | CLOSED | Each round now gathers candidate events from active literal buckets with stamp deduplication; malformed/missing index falls back to a full scan. Capacity and one-missing proofs still scan all events | Chained literal fixed-point and fallback regressions in `test_branch_and_cut` |
| A18 | Cut-derived domain loops computed floor/ceil values and only incremented counters | CLOSED | Non-applying loops and the phantom statistic were deleted. Remaining cut-row code populates the variable-bound table/implication graph, which is consumed | No `variable_bound_cut_domain_tightenings` member; implication consumers remain live |
| A19 | Cut-pool hash and aging were decorative; eviction was FIFO-like | PARTIAL | Dedup uses a support-hash bucket followed by real support/parallelism/RHS comparison. New cuts start at age 0. Serial/shared selection ages and purges transactionally | Full violation selection still scans the pool; fixed-cohort time/memory evidence is pending |
| A20 | `incumbent_quality_reject_factor=1e6` was effectively an empty quality gate | PARTIAL | The gate is live at all incumbent publication paths and catches extreme repair artifacts; zero/non-finite disables it | Its default threshold remains heuristic and must not be marketed as a correctness proof |

## B. Structural performance findings

| ID | Audit finding | Status | Current evidence | Work required to close |
|---|---|---|---|---|
| P01 | Presolve repeatedly recomputes row activities | OPEN | Full activity scans remain in native presolve | Incremental row activity state and affected-row queue; compare presolve time/reductions |
| P02 | Presolve probing copies full bound vectors and discards learned structure | OPEN | Snapshot/restore and full-vector intersection remain | Trail-based probing; persist implications and clique/domain deductions |
| P03 | Persistent node LP exists but normal tree paths clear `cached_sparse_basis` | OPEN | Resets remain in `bc_legacy_helpers.cpp`, root/tree handoff, and relaxation code | Ownership/alias audit, per-worker persistent LP, sibling-safe basis/factor reuse |
| P04 | Vendored wrapper materializes a dense basis inverse | OPEN | `basis_inverse = MatrixXd::Zero(m,m)` remains for bounded row counts | Remove default materialization; request BTRAN/FTRAN rows lazily |
| P05 | Known bound changes still call full `update_standard_form_bounds`; incremental API has zero callers | OPEN | `update_standard_form_bounds_incremental()` remains uncalled | Pass branch/propagation deltas through the node-LP boundary |
| P06 | Every open node stores dense `lb/ub` and deep-copied proof/cut payloads | OPEN | Legacy `Node::branch_child()` still copies dense domains and payload vectors | Adopt an arena plus path deltas and reconstruct domain on install/backtrack |
| P07 | Conflict propagation has no watched literals or variable-to-clause index | OPEN | Exact dedup is fixed, but propagation still scans clauses to a fixed point | Watched bound literals, activity/LBD aging, indexed wakeup |
| P08 | Reason tracking performs eager quadratic materialization and unbounded arena growth | OPEN | Existing reason construction remains broadly eager | Lazy reason handles; materialize only for requested conflict/proof output |
| P09 | Node queue uses multiple ordered containers plus vector scans/string signatures | OPEN | Exact domain identity is fixed, but the multi-index queue remains | Indexed heap/pairing heap, lazy incumbent pruning, compact domain keys |
| P10 | Parallel workers scan queues under global locks; monitor polls every 1 ms | OPEN | 1 ms/5 ms polling and queue-wide operations remain | Condition-variable/event-driven monitor and amortized global-bound updates |
| P11 | Strong branching copies full standard forms and cold-solves probes | PARTIAL | Local exact evidence, directional reliability, reuse signatures, and duplicate-sample prevention are implemented; `probe_sf = tree_base_sf` remains | Probe on the active persistent LP with iteration/objective cutoff limits |
| P12 | Fractionality is rescanned across all integer variables at multiple sites | OPEN | Repeated `fractional_branchable_indices()` calls remain | Incremental fractional set maintained with node LP solution updates |
| P13 | Root probing repeats a fixed branch and performs full-vector extraction | OPEN | The audited multi-pass root probing structure remains | Cache both branch worlds and export sparse changed-bound sets once |
| P14 | Separators allocate dense `n` vectors and rebuild row-major workspaces | OPEN | Cover/MIR/tableau paths still contain dense candidate vectors and repeated row views | Sparse cut builders and a shared separator workspace |
| P15 | Clique-table updates rebuild CSR graphs | OPEN | `add_edges()`/`add_literal_edges()` architecture is unchanged | Batched or chunked adjacency insertion with rebuild thresholds |
| P16 | Propagation uses cancellation-prone residual arithmetic and absolute tolerances | OPEN | Audited formulas remain in propagation/presolve | Incremental compensated activities and scale-aware tolerances |
| P17 | Dual proof construction is eager after incumbents | OPEN | Reduced-cost proof resolve remains enabled by default | Trigger proof rows only for an actual conflict/cutoff analysis request |
| P18 | LP data is copied through LPModel, standard form, HiGHS arrays, and result forms | OPEN | Multiple matrix layouts and `out.form = sf` remain | Persistent model ownership and view-based result/proof interfaces |
| P19 | `BCDomain::restore()` re-marks every row, losing backtrack incrementality | OPEN | Restore still loops over all rows after clearing the queue | Trail propagation marks and restore the prior queue/savepoint state |
| P20 | No incumbent-driven tree restart | OPEN | No production restart state exists; unsupported restart API was removed instead of faked | Design a real restart contract with global cuts/conflicts/pseudocost preservation |
| P21 | Early concurrent tree is hard-disabled; cut worker is normally gated off | OPEN | `allow_concurrent_tree=false`; cut worker requires tree cuts | Remove only after root-state ownership is safe and experiments justify it |
| P22 | Parallel search is not fully deterministic | PARTIAL | Turn token and consistent incumbent snapshots reduce scheduling variance; LP completion timing and floating reduction order still vary | Deterministic event ordering or explicitly documented best-effort preset |

## C. Engineering and reproducibility

| ID | Audit finding | Status | Current evidence | Work required to close |
|---|---|---|---|---|
| E01 | Entire live engine is one approximately 35k-line function | OPEN | `branch_and_cut.cpp` is still about 37k lines | Extract state-owning phases only after correctness contracts are testable |
| E02 | `BCOptions` is an approximately 230-field flat public blob | OPEN | Dead fields are gone, but about 217 fields remain and internal tuning is still public | Separate stable public policy, experimental options, and internal autotuning |
| E03 | Hidden environment configuration prevents replay | OPEN | Current census: 173 `getenv` calls and 51 literal names; only a minority route through `bc_env_options()` | One captured solve configuration, typed parsing, result provenance, no ad hoc reads |
| E04 | Documentation and names contradicted actual behavior | PARTIAL | StrictHiGHS, public LP-backend comment, presolve comment, warm-start docs, and false RINS claim are corrected | Audit all other performance documents; delete unsupported speed claims |
| E05 | Phase/P-number comments imitate a roadmap inside production APIs | PARTIAL | `TODO/FIXME/HACK/placeholder/scaffold/not implemented` census is now zero in the native scope, but many P/Phase labels remain | Rewrite comments as current contracts; move future work to design documents |
| E06 | Benchmark outcomes could be mislabeled or compare inconsistent objectives | PARTIAL | Runner validates original-space objective/status and PAR-10 classification; prior report is invalidated | Repeat full fixed cohort with hard watchdogs and solver-version provenance |
| E07 | Theory documents mixed implemented facts and plans | PARTIAL | System theory now separates Native, vendored HiGHS, and external references and states no general speed win | Recheck every version/literature statement and attach source URLs/commits |
| E08 | No current experiment supports a general Native efficiency claim | OPEN | Previous four-instance result has SCIP best aggregate; old 48-run report is stale after correctness changes | Rebuild all solvers and run repeated Native/HiGHS/SCIP cohort before any conclusion |

## Current census and non-claims

Reproduced against the working tree on 2026-08-01:

- `BCOptions`: approximately 217 extracted data fields; zero with no member
  access in `src/include/benchmark/tests` under the audit's method.
- `BCStats`: approximately 354 extracted data fields; zero with no member
  access under the same method. This checks liveness, not semantic usefulness.
- Native MILP environment access: 173 `getenv` call sites and 51 distinct
  string literals. This remains an open reproducibility defect.
- Native placeholder-word census: zero `TODO/FIXME/HACK/XXX/placeholder/
  scaffold/not implemented` hits. P/Phase roadmap labels remain and are tracked
  by E05.
- The live engine remains approximately 37k lines in one translation unit.

The project must not currently claim that Native MILP is generally faster than
HiGHS or SCIP. The latest admissible aggregate conclusion remains that SCIP was
best on the small controlled cohort; all performance evidence must be rerun
after the remediation set stabilizes.

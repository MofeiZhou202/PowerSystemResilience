# Native Dual Simplex — S3 Proposal / Certification / Commit (design)

Roadmap `native_dual_simplex_system_roadmap_2026-08-04.md` stage **S3** (§7,
§16 step 5). This is the design + incremental Class-P plan for reorganizing the
pivot transaction into three explicit phases so the performance kernel is
replaceable and the correctness boundary does not change under SIMD/task
scheduling (the S4 prerequisite). Scalar certification and the transaction
boundary must not be weakened for performance (§7, §12). S0–S2 are merged; this
work does not itself close the serial gap (see DR-1 / §18 — per-pivot cost is
near its floor on the target cases) but formalizes the boundary S4 needs.

## 1. Current `minor_iteration` mapping (solver.cpp)

One dual minor iteration already contains the three phases *interleaved*. The
mapping, with the operation and its current line region:

**Proposal** (should read solver state, write only private workspace):
- CHUZR `choose_leaving` → `leaving{row, side, delta, violation, row_ep}`.
- PRICE `multiply_AT_indexed_bfrt` → `pivot_row`.
- CHUZC/BFRT `choose_entering_bfrt` → `transaction{entering, flips, cost_shifts,…}`.
- col_aq FTRAN `indexed_ftran(column,true)` → `direction`, `column_pivot`.
- col_bfrt FTRAN `indexed_ftran_into` → `bfrt_delta`; `remaining_delta`.
- DSE `compute_dse_weights` → `edge_weight_update`.
- primal transaction → `primal_changes` (from `bfrt_delta` + `direction`).

**Certification** (scalar order, no mutation):
- `pivot_row.finite()`.
- BFRT postcondition (dual feasibility over `pivot_row` support).
- `column_pivot` finite and nonzero.
- `remaining_delta` sign within the rounding envelope.
- `primal_changes` finiteness.
- factor-update precondition (pivot validity) — **currently only enforced
  inside `update_indexed`, after basis/move mutation**.

**Commit** (state mutation):
- `PivotStateGuard` applies move/basis flips — **before** the factor update.
- `state.basis[leaving.row] = entering.col`; `factor->update_indexed(...)` —
  may fail → `PivotStateGuard` rolls back.
- cycle signature, `edge_weight`, `basic`, `x_basic` + `refresh_leaving_heap`,
  `reduced_costs`, `cost_shifts`, `record_cycle_arrival`, devex, statistics.

## 2. The single impurity to remove

Commit is entangled with certification at exactly one place: the **factor
update** both *validates* (pivot magnitude / stability) and *mutates* the
factorization, and the move/basis mutation is applied *before* it, so a failed
update requires `PivotStateGuard` to roll back published state. Everything else
is already proposal-then-certify-then-commit in order.

The clean S3 target lifts the factor-update **precondition** (the pivot-validity
check, already computed as `column_pivot` and the stability tolerance) into the
certification phase, so that when commit runs, every check has passed and no
rollback path is reachable:

```text
PivotProposal propose(state, scratch);   // reads state; writes only workspace
bool          certify(state, proposal);  // scalar checks; no mutation
void          commit (state, proposal);  // atomic; only after certify == true
```

`PivotStateGuard` then degenerates from a rollback mechanism to a debug
assertion that commit is total.

## 3. Class-P invariants (must hold at every step)

- Every floating-point operand, operation **order**, and branch outcome is
  unchanged: pivots, rebuilds, phase counts, BFRT flip/shift, termination class
  byte-identical to the S1 baseline (d2q06c 5609/31, degen3 2082/14, 25fv47
  2181/14).
- Certification runs in the **same scalar order** as today; no check is dropped,
  reordered across a value it guards, or weakened.
- The proposal writes only private/thread-local workspace; `commit` is the only
  writer of published `State` fields.
- No new fallback, cleanup, or phase-transition class appears in telemetry.

## 4. Incremental plan (each a separate Class-P commit)

1. **Proposal aggregate.** Introduce a `PivotProposal` view bundling the already
   computed outputs (`transaction`, `direction`, `column_pivot`, `bfrt_delta`,
   `remaining_delta`, `edge_weight_update`, `primal_changes`). Pure aggregation;
   no computation moves. Validates the naming boundary compiles and stays
   bit-identical.
2. **Extract `certify`.** Move the certification checks (§1) into one function
   consuming the proposal, in the same order, returning a failure string.
   Pure code motion of comparisons/finiteness (no FP reduction reordering).
3. **Hoist the factor-update precondition.** Compute the pivot-validity /
   stability check that `update_indexed` performs, in `certify`, before any
   mutation. Then reorder commit so move/basis mutation follows the (now
   pre-certified) factor update. This is the one step that changes *when*
   mutation happens (not any value); validate bit-identical pivots with extra
   care, and keep `update_indexed` returning its status as a defensive assert.
4. **Extract `commit`.** Move the atomic mutation block into `commit(state,
   proposal)`; `PivotStateGuard` becomes a debug-only totality assertion.
5. **Re-record the path contract** and the [DS-*] trace; confirm the 24×3 A/B/A
   stays 72/72 with byte-identical non-time records.

## 5. What S3 does and does not buy

- Does: a stable proposal/certification/commit boundary; the perf kernel
  (proposal) becomes replaceable (SIMD, and S4's per-worker private scratch +
  post-join scalar certification) without touching the certification/commit
  contract.
- Does not: reduce serial per-pivot cost on the target cases (DR-1). The A/B/A
  is expected to be within noise; retention is on the structural boundary, not
  a wall claim. Any wall regression beyond noise blocks the step.

## 6. Non-goals / forbidden (§12)

No weakening of scalar certification or the transaction boundary; no
case/density/wall selector; no PRICE/CSC switch, BFRT RHS layout retry,
CHUZR/PRICE fusion, or prevalidated reduced-cost stream introduced under cover
of the refactor.

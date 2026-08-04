# Native Dual Simplex — S2 Factor-Resident Pivot Workspace (design)

Roadmap `native_dual_simplex_system_roadmap_2026-08-04.md` stage **S2**, §6 and
§16 step 3 ("design the factor-owned workspace lifetime/ownership API, then do
the independent-kernel benchmark"). This is the design deliverable only; no
production migration is promoted before its independent-kernel evidence and the
§6.3 completion gate. Prerequisite S0 (bound-domain) and S1 (measurement
contract) are merged (`a7ee420`, `d9b26b2`).

## 1. Goal (roadmap §6.1)

Generate each per-pivot vector once inside the factor backend, keep it resident,
and let every consumer read it directly — removing export/reimport, repeated
clear/scatter, sparse hash lookups and cross-layer temporaries. The value is
proven by fewer **bytes and instructions across the whole pivot lifecycle**, not
by removing one local allocation.

Factor-owned named workspaces: `row_ep | col_aq | col_dse | col_bfrt`.

## 2. Current per-pivot dataflow (ground truth)

One dual minor iteration is `minor_iteration` in
[solver.cpp](../src/engine/kernel/lp_kernel/native_dual/solver.cpp). The four
vectors, their producers, consumers, transfers and invalidation:

| Vector | Produced by | Backing today | Consumers | Redundant motion | Invalidated |
|---|---|---|---|---|---|
| `row_ep` (leaving-row BTRAN, ρ) | `choose_leaving` → `leaving.row_ep` (BTRAN in pricing/CHUZR) | native `IndexedVector`, **moved** into `Leaving`; dense scatter built lazily on first exact bound eval | PRICE `multiply_AT_indexed_bfrt`; DSE reference weight | move-in from factor evidence; lazy dense re-scatter of a vector the factor just held | next `choose_leaving` |
| `col_aq` (entering-col FTRAN, α_q) | `state.factor->indexed_ftran(entering_rhs)` → `direction` | native `IndexedVector` | pivot-identity verify (`column_pivot`); `compute_dse_weights`; primal update (`add_primal_delta(direction…)`); edge-weight loop; `update_indexed` | scatter of `direction` into `g_ds_scratch.primal_delta` (stamped) | next pivot |
| `col_dse` (DSE FTRAN) | inside `compute_dse_weights` via `indexed_ftran_at_captured_pattern` | **reads HFactor dense backing directly** at `col_aq`'s captured pattern | edge-weight update | none — this is the S2 pattern already (speedup-doc L4a) | next pivot |
| `col_bfrt` (BFRT flip-RHS FTRAN) | `state.factor->indexed_ftran(transaction.bfrt_rhs)` → `bfrt_delta` | native `IndexedVector`, **allocated fresh per pivot** (`detail::IndexedVector bfrt_delta;`) | leaving-row check `bfrt_delta.at(row)`; primal update (`add_primal_delta(bfrt_delta…)`) | per-pivot allocation + `.at()` hash lookup + scatter into `primal_delta` | end of iteration |

Native-owned per-pivot scratch: `g_ds_scratch` (`MinorScratch`, thread-local:
`entering_column`, `primal_changes`, `primal_delta`+`primal_stamp`+`primal_touched`,
`edge_weight_update`, flip/shift stamps), `pivot_row_storage`,
`bfrt_active_position`. Most are already capacity-reused; `bfrt_delta` is the
one true per-pivot allocation.

Observation: `col_dse` proves the pattern works numerically and is faster
(HFactor `HVector::array` read on a captured support, no packed export / no
`dense_rho` clear / no selector scatter). S2 generalizes it to `row_ep`,
`col_aq`, `col_bfrt`.

## 3. Redundant data motion S2 targets

1. `col_bfrt` per-pivot `IndexedVector` allocation + `.at(row)` coordinate hash
   lookup + full scatter into `primal_delta`.
2. `row_ep` move-out of factor evidence into `Leaving`, then a lazy dense
   re-scatter for exact bound evaluation of a vector the factor already held
   densely.
3. `col_aq` scatter into the stamped `primal_delta` accumulator when the factor
   already holds α_q densely on its support.
4. Two authorities for the same numeric fact (factor dense array vs native
   packed `IndexedVector`) that must be kept in sync.

## 4. Factor-owned workspace model

Each of the four names is a slot the `HFactorBackend` owns for the lifetime of
one factor generation and rewrites in place each pivot. Each slot exposes,
simultaneously and over the same storage:

- **dense coordinate view** — `const double* dense()` addressable by row/col;
  entries off the support are exact zero and stay zero (owner clears only the
  touched support on recycle).
- **ordered packed support** — `span<const int> support()` +
  `span<const double> values()` in the factor's stable produced order (the
  numerical-contract order; Class P must not reorder).
- **generation/lifetime token** — `WorkspaceToken { factor_generation; pivot_seq; slot; }`;
  a consumer read asserts the token so a stale view is a hard error, not silent
  corruption.
- **ownership state** — `Proposal` (producer writable) → `Certified` (read-only)
  → `Committed`/recycled. Certification and commit never observe a writable slot.

Dense and packed views reference one storage; there is no second authoritative
copy and no producer→consumer copy.

## 5. Proposed lifetime/ownership API (sketch)

On `HFactorBackend` (wrapped by `BasisFactor`), additive to today's
`indexed_ftran`/`indexed_btran`/`indexed_ftran_at_captured_pattern`:

```cpp
enum class PivotSlot { RowEp, ColAq, ColDse, ColBfrt };

// A borrow of a factor-resident slot. Non-owning; valid only while the token
// matches the backend's current (generation, pivot_seq, slot).
struct ResidentView {
  const double*        dense;        // length m, zero off support
  const int*           support;      // stable produced order
  const double*        support_val;  // parallel to support
  int                  support_size;
  WorkspaceToken       token;        // asserted on every read
};

// BTRAN the leaving row into the RowEp slot and return a borrow.
ResidentView btran_into(PivotSlot slot, const IndexedVector& rhs, bool verify);
// FTRAN the entering column / BFRT RHS into a slot and return a borrow.
ResidentView ftran_into(PivotSlot slot, const IndexedVector& rhs, bool verify);
// Read an already-produced slot at another slot's captured support
// (generalizes indexed_ftran_at_captured_pattern; col_dse uses this today).
ResidentView view_at_support(PivotSlot slot, PivotSlot pattern_of) const;

bool token_live(const WorkspaceToken&) const;   // consumer guard
void recycle_pivot();                            // clears only touched supports
```

Consumers change from owning an `IndexedVector` to borrowing a `ResidentView`:
`multiply_AT_indexed_bfrt` reads `row_ep` via its view; the primal transaction
reads `col_aq`/`col_bfrt` dense-on-support instead of scattering them into
`primal_delta`; pivot verification reads `col_aq.dense[row]` directly;
`.at(leaving.row)` on `col_bfrt` becomes `dense[leaving.row]` (no hash).

## 6. Class P constraints (roadmap §6.2)

- Packed `support()` order == today's produced order (the reduction/merge order
  is numerical contract). No reordering under Class P.
- One storage backs both views; no bidirectional copy.
- Slot capacity reused across the solve/factor lifetime; cold and warm solves do
  not reallocate.
- The backend exposes factor results and stable views only; no algorithmic
  decision (side, tolerance, selection) leaks into it.
- Serial correctness + independent-kernel evidence precede any executor wiring
  (S4).

## 7. Independent-kernel benchmark plan (§6.3 gate)

A standalone kernel (as with the BFRT SIMD kernel in
`native_dual_simplex_kernel_speedup_plan.md`) replays a recorded pivot cohort:
`(support, values, consumer set)` drawn from the **production distribution**
(d2q06c/degen3/25fv47 densities: row_ep ~25–46% of m, pivot-row ~26–48% of n —
from the S1 phase baseline), not uniform synthetic. It runs the current
move-in/scatter path vs the resident-view path and reports, per §13.3:

- dynamic instructions (analytic basic-block model + audited count) — must drop;
- effective read/write bytes (array model) — must drop;
- both must drop *together* for the structural gate to pass.

Then the full 24×3 A/B/A must stay 72/72 accurate with an identical timing-
stripped path record (Class P), and the wall change must agree in sign with the
kernel cost model. Perf-counter memory-stall separation is added when this gate
first needs the hardware confirmation (S1 deferred item), on the platform chosen
then.

## 8. Migration order (lowest risk → highest)

1. **`col_bfrt`** first: it is the only true per-pivot allocation and uses a
   `.at()` hash lookup — a resident dense-on-support view removes an allocation,
   a hash, and a scatter with a bit-identical primal transaction. Smallest
   surface, clearest Class-P argument.
2. **`row_ep`**: replace the move-into-`Leaving` + lazy dense scatter with the
   factor-resident BTRAN view consumed by PRICE.
3. **`col_aq`**: read α_q dense-on-support in the primal transaction and pivot
   verification instead of scattering into `primal_delta`.
4. **`col_dse`**: already resident — fold it into the same `ResidentView` API for
   uniformity (no numeric change; documents the reference case).

Each step is a separate Class-P change with its own independent-kernel evidence
and 24×3 A/B/A; none is promoted on wall time alone.

## 9. Completion gate (roadmap §6.3) — tracking

- [ ] Owner/producer/consumer/invalidation table asserted in code for each slot
  (this doc §2 is the design; implementation adds the `WorkspaceToken` asserts).
- [ ] Class P path contract identical (timing-stripped report byte-identical).
- [ ] Independent kernel: dynamic instructions **and** bytes both drop.
- [ ] Full 24×3 A/B/A no correctness regression; wall change agrees with the
  kernel cost model.

## 10. Measured result — row_ep PRICE resident view (independent kernel)

Harness: [native_dual_price_resident_benchmark.cpp](../benchmark/native_dual_price_resident_benchmark.cpp)
(target `native_dual_price_resident_benchmark`, self-contained). It replays
`dot_error_bound` over production-density fixtures (d2q06c-shaped m=1759, n=6423,
row_ep support ~40% of m) comparing the current *materialize* path
(`dense_row_ep` = O(m) clear + O(support) scatter, then read) against the S2
*resident* path (read the factor's dense BTRAN backing directly). Both checksums
are **bit-identical**.

Crossover vs candidates dotted per pivot (M4, Release):

| candidates/pivot | materialize/resident | note |
|---:|---:|---|
| 2 | 1.083x | resident wins when almost nothing is dotted |
| 8 | 1.078x | |
| 32 | 1.025x | |
| 128 | 1.006x | |
| 6423 (all) | 1.000x | fully amortized |

Byte model: the resident path removes `m·8 + |support|·16 ≈ 25.7 kB` per pivot
(the clear + scatter). **But** `dense_row_ep` is built once per pivot and
amortized over every certified candidate dot, and the shared dot-scan dominates:
on the representative large cases the BFRT scan certifies on the order of
hundreds of candidates per pivot (telemetry ~957 BFRT candidates on d2q06c), i.e.
the ≥128 regime, where the removed 25.7 kB buys ≲0.6 % — below wall resolution.

**Conclusion: `diagnostic-only`.** The row_ep materialize is already well
amortized; the resident view removes real bytes that do not reach wall time on
the target cases, confirming the roadmap rule that a byte reduction alone is not
a sufficient gate (§13.3). A resident view is only worth the hot-loop risk where
few candidates are certified per pivot (small degenerate models), which are not
the per-pivot-cost bottleneck. The harness is retained to gate future workspace
proposals (e.g. col_aq) against this same amortization test before any hot-loop
change.


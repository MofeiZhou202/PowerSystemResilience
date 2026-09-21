---
name: theory-guided-coding
description: Enforces theory-first development for numerical and algorithmic code (solvers, linear algebra, optimization kernels, performance work). Use BEFORE implementing or optimizing any algorithmic/numerical code, when proposing performance improvements or algorithm changes, and AFTER benchmarking a change. Requires a theoretical rationale with cost model and quantitative prediction before coding, literature/equation references at code sites, zero incomplete implementations, and a mandatory re-derivation protocol when measured performance deviates far from the prediction.
license: Personal methodology skill (Tianyang Zhao)
metadata:
  version: "1.0.0"
  triggers: implement algorithm, optimize, performance, numerical, solver, kernel, refactor hot path, benchmark
---

# Theory-Guided Coding

Development contract for numerical/algorithmic code. Code is the executable
form of a derivation — if the derivation is missing or wrong, the code is
wrong even when it compiles and the tests pass.

## The five rules

1. **Theory leads, code follows.** Every implementation or modification is
   guided by an explicit model or algorithm (paper, textbook, derivation
   document), never by guessing at the code level.
2. **References at the site.** Nontrivial formulas, update rules, tolerances,
   and constants in code carry a citation.
3. **Complete implementations only.** Unfinished paths and fabricated values are treated as
   deception: they make unfinished work look finished.
4. **Rationale gate before implementation.** The theoretical reason and the
   quantitative prediction are written down and confirmed BEFORE code changes.
5. **Mismatch triggers re-derivation.** If measured performance is far from
   the prediction, the theory is re-investigated first — the mismatch is data
   about the derivation, not an excuse to tweak code randomly.

---

## Phase 0 — Theoretical rationale (REQUIRED before any code)

Produce this block in the conversation, design doc, or PR description. Do not
start implementing until every field is filled with specifics.

```
RATIONALE
Model/algorithm: <the mathematical object implemented or changed,
                  e.g., Forrest–Tomlin update, Mehrotra predictor–corrector>
Claim:           <what the change does, stated as a theorem-like sentence>
Cost model:      <complexity AND machine model,
                  e.g., O(nnz) bandwidth-bound at 12 B/nnz, or BLAS-3 O(n^1.5)>
Prediction:      <quantitative expected effect,
                  e.g., "PRICE −20% on d2q06c; fleet geomean +2%">
Assumptions:     <sparsity regime, conditioning, input distribution, scale>
References:      <paper/book with section/equation numbers; baseline code;
                  internal derivation docs>
Validation:      <exact benchmark command, metric, acceptance threshold —
                  fixed BEFORE seeing results>
```

An "optimization" with no predicted magnitude is not ready to implement.
A field marked "N/A" requires a one-line justification.

## Phase 1 — Implementation rules

- **Cite at the site.** Every nontrivial formula, update rule, tolerance, or
  magic constant gets a reference comment that will outlive the PR:
  ```cpp
  // Goldfarb & Reid (1977), eq. (3.2); derivation in docs/dse_theory.md §4.
  // Tolerance from Gill et al. (1987) §5: pivot growth bound 1e5.
  ```
  Cite constraints and derivations — do not narrate what the next line does.
- **Whole implementations only.** Forbidden in delivered code: unfinished-work
  markers (the exact patterns are listed in the self-check below), empty
  branches that silently succeed, and fake/plausible return values. If a case
  cannot be handled now, either cut the scope explicitly (documented, and the
  path rejects loudly at runtime) or do not deliver.
- **Fail loudly.** An unhandled path must abort or return an explicit error —
  never fabricate a number.
- **Self-check before finishing** — run on the diff and require zero hits:
  ```bash
  marker_pattern='TO''DO|FIX''ME|X''XX|HA''CK|place''holder|not.?imple''mented|dummy va''lue|st''ub'
  git diff | grep -inE "^\+.*(${marker_pattern})"
  ```

## Phase 2 — Empirical validation

- Run the validation protocol from Phase 0 exactly as written.
- Report **measured vs predicted side by side**. Never report success without
  numbers, and never move the acceptance threshold after seeing results.
- Preserve reproducibility: record the benchmark command, machine, build
  flags, and commit hash next to the numbers.

## Phase 3 — Mismatch protocol

Mandatory when the measured effect deviates far from the prediction
(default trigger: wrong sign, or |measured − predicted| > 50% of predicted).
Investigate IN THIS ORDER and document which suspect it was:

1. **Implementation infidelity** — does the code compute the derived
   expression? Check with a small-case oracle or unit identity
   (e.g., B·(B⁻¹x) = x to tolerance).
2. **Machine/cost-model error** — wrong resource assumed: FLOP-bound vs
   bandwidth/latency-bound, cache effects, index width, allocator traffic.
3. **Assumption violation** — the input regime differs from the theory's
   assumptions: density, degeneracy, conditioning, problem scale.
4. **Theory error** — re-derive. Fix the derivation document BEFORE touching
   the code again.

The outcome (which suspect, and the corrected understanding) goes back into
the derivation document. A mismatch that is "explained away" without a
documented cause is an open defect.

## Red flags — stop and restart the phase

- Writing code before the RATIONALE block exists.
- A constant or tolerance with no reference and no derivation.
- A temporary incomplete implementation added to make tests pass.
- A benchmark below prediction rationalized without re-derivation.
- Reporting an improvement without the predicted number next to it.

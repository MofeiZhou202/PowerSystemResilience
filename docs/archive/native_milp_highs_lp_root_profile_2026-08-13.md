# Native MILP HiGHS-LP Root Profile

## Rationale

Model/algorithm: the native branch-and-cut tree with the repository's
HiGHS-style root separation fixed point enabled when HiGHS is the node-LP
kernel.  The fixed point repeatedly applies implied-bound propagation,
tableau/path/mod-k separation, cutpool selection, and LP reoptimization until
no rows move the state or the smooth-progress stall rule fires.  A valid cut
that removes only part of a degenerate optimal face is retained even when its
first reoptimization has zero objective lift.

Claim: `native-highs-lp` in the MIPLIB benchmark exercises the native HiGHS-LP
production root profile instead of the generic native profile.  This changes
only benchmark option wiring: it does not enable `strict_highs_mip_contract`
and therefore does not replace the native tree with the upstream HiGHS MIP
state machine.

Cost model: on `sp150x300d`, the first five native rounds currently cost about
48 ms and stop after rejecting 18 zero-lift rows.  Retaining such rows permits
at most the existing three smooth-progress stall rounds; predicted root
separation is below 0.25 seconds.  With the verified warm-start incumbent,
incumbent-search-only feasibility pump work is already gated off, so total
one-node runtime is predicted below 0.50 seconds.

Prediction: on the fixed `sp150x300d` optimum-witness command, the reduced
root bound is at least `39.0` and at most the known optimum `40.0`, runtime is
below `0.50 s`, and the incumbent audit passes.  On the five-second
`sp150x300d,p200x1188c,neos-1122047` cohort, no completed native incumbent
fails audit and `sp150x300d` relative gap is at most `0.02` (baseline
`0.075`).

Assumptions: the HiGHS LP backend returns a valid changed basic solution after
zero-lift rows alter a degenerate face; root cut rows remain globally valid;
and the existing `has_incumbent` gates suppress feasibility-pump work when the
oracle incumbent is present.

References: Achterberg (2007), Sections 4.1-4.2; HiGHS
`HighsSeparation::separate`, `HighsCutPool::separate`, and
`HighsLpRelaxation::addCuts`; prior mismatch analysis in
`docs/native_milp_degenerate_cutpool_fixed_point_2026-08-13.md`.

## Validation Contract

Baseline commit `1b51f0bd8caa`, macOS ARM64, Release `-O3 -DNDEBUG`, one
thread.  Run `test_branch_and_cut`, `test_milp_solver`, the fixed
`sp150x300d` optimum-witness command from
`docs/native_milp_presolve_varbound_coordinates_2026-08-13.md`, and the fixed
three-instance five-second cohort.  A root bound above 40, a failed incumbent
audit, runtime above 0.50 seconds on the witness run, or `sp150x300d` cohort
gap above 0.02 triggers the mismatch protocol before another algorithm edit.

## Measurement And Rejection

With the verified optimum supplied as an incumbent, the profile reached a
reduced root bound of `38.89384065`, proved the integer objective at one node,
and finished in `0.265 s`.  The incumbent passed its original-model audit.
This met the runtime and end-to-end proof targets; the raw bound missed the
`39.0` prediction by `0.10615935`, only 3.4% of the predicted lift.

The no-hint cohort exposed a correctness failure, however.  The solver found
an audited incumbent of 70 and published an original-space lower bound of 70,
although the independently verified optimum is 69.  An audit-hint-only run
showed that the objective-69 witness remains feasible through the root at
reduced bound `38.89384065`.  The false lower bound is therefore introduced by
the downstream tree-domain/cutpool behavior activated by the profile rather
than by the root affine transport.

Mismatch protocol:

1. Implementation infidelity: benchmark wiring did enable the intended root
   policy, and zero-lift rows were retained.  The resulting profile was not
   root-local, however; `auto_highs_root_pipeline` also changes downstream
   tree behavior.
2. Machine/cost-model error: not implicated; the witness run remained below
   the 0.50-second ceiling.
3. Assumption violation: confirmed.  The rationale assumed the option selected
   only a root lifecycle, but its behavioral scope extends into the tree.
4. Theory error: a root-valid fixed-point argument cannot certify unverified
   downstream domain/cutpool state.  The benchmark profile change is rejected.

The production benchmark remains on the generic native-tree policy until a
separate derivation isolates and audits the tree effects of the HiGHS-style
root artifacts.

# Native MILP Presolve Variable-Bound Coordinates

## Defect

Native root setup extracts variable-bound rows from the original MILP before
HiGHS presolve.  After adopting the HiGHS-reduced model, it replayed those rows
using their original column numbers as reduced column numbers.  On
`sp150x300d`, this retained 176 records from a 600-column model in a 476-column
model.  A verified optimal solution exposed invalid implications such as
reduced column 148 being forced below zero when its feasible value is 1593.

This invalid source graph then generated transformed cuts and raised the
reduced LP bound above the known reduced optimum of 40.  The apparent stronger
tree was therefore not a tree-quality improvement; it was an invalid
coordinate replay.

## Rationale

Model/algorithm: exact affine transport of a variable-bound inequality through
the retained HiGHS postsolve map.  For

```text
x_t <= a x_z + b
x_t = s_t y_t + c_t
x_z = s_z y_z + c_z
```

the reduced-space row is

```text
y_t <= (a s_z / s_t) y_z
       + (a c_z + b - c_t) / s_t,
```

with the inequality direction reversed when `s_t < 0`.  The same substitution
applies to a lower bound.  A record is rejected when either endpoint was
eliminated, either forward transform is unavailable, a scale is zero, or the
mapped trigger is not binary in the reduced model.

Claim: every replayed original-space variable bound denotes the same feasible
set in the HiGHS-reduced coordinates.  No original index is interpreted as a
reduced index without an explicit retained map.

Cost model: one original-to-reduced lookup and constant arithmetic per source,
`O(number of variable-bound sources)` time and no asymptotic storage increase.
`sp150x300d` has 300 original sources, so the predicted setup cost is below one
millisecond.

Prediction: the verified HiGHS optimum remains feasible in the native reduced
root; the reduced root bound never exceeds the known reduced optimum `40`; the
final root bound is at least `39.0`; and one-node root time remains below one
second.  Every published incumbent passes the original-model audit.

Assumptions: retained HiGHS maps obey
`x_original = scale * x_reduced + constant`; surviving trigger variables are
binary after mapping; and rejected eliminated endpoints are not reconstructed
as multi-column affine expressions.

References: HiGHS `HighsPostsolveStack::linearTransform` and
`HighsImplications::VarBound`; Achterberg (2007), Sections 4.1-4.2 for the
validity requirement on globally separated cuts and implied-bound transforms.

## Validation Contract

Baseline commit `1b51f0b`, macOS ARM64, Release `-O3 -DNDEBUG`, one thread.

```text
./tests/miplib2017_benchmark \
  --data-dir tests/data/miplib2017/benchmark \
  --solu tests/data/miplib2017/miplib2017-v36.solu \
  --solvers native-highs-lp --case sp150x300d \
  --time-limit 30 --hard-timeout-grace 2 --max-nodes 1 --seed 0 \
  --native-probe-max 0 --native-primal-hint /tmp/sp150_highs_hint.json \
  --native-verbose
```

After the fixed oracle run, execute `test_branch_and_cut`, `test_milp_solver`,
and the five-second `sp150x300d,p200x1188c,neos-1122047` cohort.  A wrong-sign
bound change or a root bound above 40 triggers the mismatch protocol before
any further algorithm change.

## Measured Result And Mismatch Re-Derivation

Measured on macOS ARM64 at commit `1b51f0bd8caa`, Release
`-O3 -DNDEBUG`, with the validation command above:

| Quantity | Predicted | Measured |
|---|---:|---:|
| Verified reduced optimum accepted | yes | yes, objective `40.0` |
| Invalid global implication witness | none | none |
| Reduced root bound upper limit | `40.0` | `35.8878721968` |
| Reduced root bound strength | at least `39.0` | `35.8878721968` |
| Root processing time | below `1 s` | `0.102 s` |
| Original incumbent audit | pass | pass, max row violation `1.4e-14` |

The implementation-fidelity checks passed: the affine identity has direct unit
coverage for positive and negative target scales, the HiGHS optimum projects
to reduced objective 40, and the former invalid implication witness is absent.
The machine/cost prediction also passed: mapping 300 sources was negligible
within a 102 ms root.  The mismatch is therefore an assumption violation, not
a reason to alter the transport formula.  Of 300 original sources, 195 mapped
exactly, 95 had eliminated targets, and 10 had eliminated triggers.  All 195
mapped sources were rejected as non-improving because the retained HiGHS
variable-bound table already contained 282 stronger or equal records
(`accepted=0`).  Correct coordinate replay removes invalid strength but cannot
reproduce HiGHS' first-root cut closure by itself.

The corrected conclusion is narrower: affine transport is a correctness
prerequisite, while the remaining `39.7029215963 - 35.8878721968 =
3.8150493995` reduced-bound deficit must be attributed to separation and cut
selection differences.  Further work must compare valid root cuts against the
HiGHS 16-round trace; changing variable-bound replay again requires a new
derivation and oracle.

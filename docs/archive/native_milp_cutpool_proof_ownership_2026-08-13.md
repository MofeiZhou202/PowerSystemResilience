# Native MILP Cutpool Proof Ownership

## Rationale

Model/algorithm: two-stage ownership for generated cutting planes.  A
separator candidate first belongs to the root cutpool.  It becomes an LP-owned
global proof row only after cutpool selection, successful root LP append, and
monotone reoptimization.  Dormant candidates remain available to the root
selection lifecycle but are not exported to tree-domain propagation.

Claim: the native tree consumes only transformed rows that participated in a
successful root LP certificate.  A generated-but-never-admitted candidate
cannot tighten or prune a tree node.

Cost model: the change is a filter over the existing root pool,
`O(number of root candidates)` time and no new storage.  On `sp150x300d`, 138
rows were admitted while 741 additional dormant rows were exported to the
global domain pool.  The prediction is unchanged root separation time and
lower tree propagation work.

Prediction: with the HiGHS-LP root profile active, the fixed optimum-witness
run retains a reduced root bound of at least `38.8`, proves at one node, runs
below `0.50 s`, and passes the original audit.  The no-hint five-second run
never publishes an original-space lower bound above the known optimum 69; if
it claims optimality, the independently audited objective is 69.  The fixed
three-instance cohort has no completed-incumbent audit failure.

Assumptions: root LP reoptimization is a necessary proof-ownership gate;
admitted transformed rows are globally valid; and dormant rows are not needed
to reproduce the measured root bound because they are absent from the live
root LP by definition.

References: Achterberg (2007), Sections 4.1-4.2; HiGHS
`HighsCutPool::separate`, `HighsLpRelaxation::addCuts`, and the LP-owned age
state in `HighsCutPool`; mismatch analysis in
`docs/native_milp_highs_lp_root_profile_2026-08-13.md`.

## Validation Contract

Baseline commit `1b51f0bd8caa`, macOS ARM64, Release `-O3 -DNDEBUG`, one
thread.  Run `test_branch_and_cut`, `test_milp_solver`, the fixed hint and
no-hint `sp150x300d` commands, and the fixed five-second cohort.  Any bound
above the known optimum, incumbent audit failure, witness runtime above 0.50
seconds, or witness root bound below 38.8 triggers re-derivation before
another solver edit.

## Validation Mismatch: Final Seeding Is Not The Ownership Boundary

The first build of the ownership predicate was measured at baseline commit
`1b51f0bd8caa` on macOS ARM64, Release `-O3 -DNDEBUG`, one thread.  The fixed
optimum-witness command produced reduced root bound `38.89384065` versus the
prediction `>= 38.8`, wall time `0.2612 s` versus the prediction `< 0.50 s`,
and passed the original-space incumbent audit.  It did not produce the
predicted one-node proof: the solve stopped at the one-node limit with gap
`0.0277`.

The mismatch is implementation infidelity, the first suspect required by the
re-derivation protocol.  The new final-seeding predicate reported 138
LP-owned rows as duplicates and skipped 745 provisional rows, while the
global tree cutpool already contained 879 rows.  Earlier
`root_domain_closure` scans therefore crossed the same ownership boundary
before final seeding.  Filtering only `seed_root_transformed_cutpool_domain_rows`
does not implement the claimed two-stage ownership model.  The corrected
implementation must enforce the predicate at every transformed-root-row
export into tree-global propagation, then repeat the unchanged validation
contract above.

Code inspection identified the concrete gate error: `native-highs-lp` enables
`auto_highs_root_pipeline` without enabling the full strict HiGHS contract,
while generated transformed rows were exported whenever
`!strict_highs_root_fixed_point`.  The corrected export condition retains
rows privately when either strict fixed-point mode or the automatic
HiGHS-style pipeline is active.  Root LP selection and reoptimization are
unchanged; final seeding remains the sole tree-global export for the automatic
pipeline and applies `alive && in_root_lp`.

## Accepted Validation

The corrected implementation was measured at baseline commit
`1b51f0bd8caa` on macOS ARM64, Release `-O3 -DNDEBUG`, one thread.

Build command:

```sh
cmake --build build/macos-release \
  --target test_branch_and_cut test_milp_solver miplib2017_benchmark -j4
```

Witness, no-hint, and cohort commands:

```sh
./tests/miplib2017_benchmark \
  --data-dir tests/data/miplib2017/benchmark \
  --solu tests/data/miplib2017/miplib2017-v36.solu \
  --solvers native-highs-lp --case sp150x300d \
  --time-limit 30 --hard-timeout-grace 2 --max-nodes 1 --seed 0 \
  --native-probe-max 0 --native-primal-hint /tmp/sp150_highs_hint.json \
  --native-verbose

./tests/miplib2017_benchmark \
  --data-dir tests/data/miplib2017/benchmark \
  --solu tests/data/miplib2017/miplib2017-v36.solu \
  --solvers native-highs-lp --case sp150x300d \
  --time-limit 5 --hard-timeout-grace 2 --max-nodes 5000 --seed 0

./tests/miplib2017_benchmark \
  --data-dir tests/data/miplib2017/benchmark \
  --solu tests/data/miplib2017/miplib2017-v36.solu \
  --solvers highs-mip,native-highs-lp \
  --case sp150x300d,p200x1188c,neos-1122047 \
  --time-limit 5 --hard-timeout-grace 2 --max-nodes 5000 --seed 0
```

The fixed optimum-witness command produced reduced root bound `39.4048`
versus the prediction `>= 38.8`, completed the proof at the root with zero
tree nodes, took `0.4660 s` versus the prediction `< 0.50 s`, and passed the
original-space audit with maximum row violation `1.4e-14`.  Of 772 generated
rows, 125 LP-owned rows crossed into the tree pool and 647 provisional rows
were withheld.

The fixed no-hint command produced objective `69`, published lower bound
`68.99600000000001` versus the hard ceiling `69`, completed with the requested
relative gap in 116 nodes and `0.2993 s`, and passed the independent audit
with maximum row violation `1.58e-12`.

In the fixed five-second cohort, all completed native incumbents passed their
independent audits.  HiGHS solved all three cases at one node.  Native solved
`sp150x300d` in 116 nodes, reached a `0.75` relative gap on `p200x1188c` after
149 nodes, and hit the seven-second hard watchdog on `neos-1122047`.  The
proof-ownership correction therefore closes the invalid-bound defect and
preserves the predicted focused root strength, but it does not close the
broader tree-quality and runtime gap to HiGHS.

# Native MILP Reliability Branching In Proof-Tail Search

## Defect

Production native B&C enters proof-tail queue mode as soon as it has any
audited incumbent. The branching dispatcher then bypasses reliability
branching and selects directly from historical pseudocosts. On MIPLIB 2017
`sp150x300d`, this makes `probe_max_candidates` ineffective for the complete
tree: runs with candidate budgets 0, 3, and 8 all report zero reliability
nodes, zero strong-branch LP solves, and zero complete probe pairs.

Proof-tail mode changes queue selection, incumbent-cutoff propagation, and
dual-certificate replay. It does not change the validity of the two child LP
relaxations used by reliability branching. Consequently an audited incumbent
is not a theoretical reason to replace reliability branching with unverified
pseudocost ranking.

## Rationale

Model/algorithm: reliability branching. For a fractional integer variable
`j`, tentative down/up LP solves measure the two child dual-bound gains until
each directional pseudocost has enough observations; reliable directions use
the normalized historical estimate. Candidate quality combines both
directions, preferring a variable whose weaker child gain is large.

Claim: proof-tail search preserves the feasible sets and objectives of both
branch children, so its branching candidates remain eligible for the same
reliability evaluation as pre-incumbent nodes. Removing the proof-tail bypass
restores measured child gains without changing any pruning certificate.

Cost model: at most `2 * probe_max_candidates = 6` additional persistent
HiGHS LP resolves per unreliable shallow node, only through depth 6 and only
until the directional reliability count reaches 2. The implementation reuses
one standard form and bound transactions. On the sparse `sp150x300d` tree the
predicted additional wall cost is at most 0.9 seconds through 500 nodes.

Prediction: with candidate budget 3, `sp150x300d` must report at least one
reliability node and one complete probe pair. At 500 nodes, relative gap must
fall from 0.0978 to at most 0.0958 while runtime remains at most 3.0 seconds.
At 10 seconds it must process at least 2000 nodes and must not exceed the
0.0959 baseline gap. Every reported incumbent must pass original-model audit.

Assumptions: retained HiGHS presolve is active; the node LP basis is reusable;
the first unreliable shallow-node candidate choices have material variation
in child gains; one thread and seed zero make the comparison deterministic
apart from ordinary sub-millisecond timing noise.

References: T. Achterberg, *Constraint Integer Programming*, PhD thesis,
2007, Section 5.3 (pseudocost, strong, and reliability branching); current
implementation in `bc_branching.cpp::choose_branch_var_reliability_impl`.

## Fixed Validation

Baseline commit: `1b51f0b`, macOS ARM64, Release `-O3 -DNDEBUG`.

Tree-quality command:

```text
./tests/miplib2017_benchmark \
  --data-dir tests/data/miplib2017/benchmark \
  --solu tests/data/miplib2017/miplib2017-v36.solu \
  --solvers native-highs-lp --case sp150x300d \
  --time-limit 30 --hard-timeout-grace 2 --max-nodes 500 --seed 0 \
  --native-probe-max 3 --native-probe-reliability 2
```

Wall-time command:

```text
./tests/miplib2017_benchmark \
  --data-dir tests/data/miplib2017/benchmark \
  --solu tests/data/miplib2017/miplib2017-v36.solu \
  --solvers native-highs-lp --case sp150x300d \
  --time-limit 10 --hard-timeout-grace 2 --max-nodes 20000 --seed 0 \
  --native-probe-max 3 --native-probe-reliability 2
```

Pre-change measurements:

- 500-node run: 2.139 seconds, gap 0.0978, audited incumbent.
- 10-second run: 9.018 seconds, 2434 nodes, gap 0.0959, audited incumbent.
- Both runs: zero reliability nodes, candidates, probe LP solves, and complete
  probe pairs.

The full `test_branch_and_cut` and `test_milp_solver` binaries must also pass.

## First Measurement And Re-Derivation

Removing the proof-tail bypass restored the intended algorithm: the 500-node
run used 36 reliability nodes, 68 strong-branch candidates, and 136 probe LP
solves, including 135 persistent resolves. There were zero bound-transaction
failures and zero measured-pair selection regret. Runtime was 2.205 seconds,
within the predicted 3.0-second maximum.

The tree-quality prediction failed with the wrong sign. At 500 nodes the gap
was 0.0984, worse than the 0.0978 baseline and the predicted maximum 0.0958.
Both runs had the same audited incumbent objective, 102; the certified best
bound changed from 94.8639 to 94.8180.

Mismatch investigation:

1. Implementation infidelity: not indicated. The probe counters prove the
   bypass was removed, persistent bound transactions rolled back successfully,
   and every fully measured candidate selection had zero local score regret.
2. Machine/cost-model error: not indicated for the quality miss; wall overhead
   was only 0.066 seconds and remained inside prediction.
3. Assumption violation: confirmed. The prediction assumed maximizing the
   local two-child score would lift the global minimum live-node bound after a
   fixed number of expansions. In proof-tail mode the queue always pops the
   globally smallest bound. A locally stronger split can create a different
   distribution of descendants without increasing that minimum frontier at
   500 nodes.
4. Theory error: reliability branching remains locally valid, but local
   strong-branch score is not a theorem about fixed-node best-bound progress.

No probe-count or reliability-threshold tuning follows from this mismatch.
Any further change must model the proof-tail queue/frontier interaction rather
than assume local child gains directly predict global tree closure.

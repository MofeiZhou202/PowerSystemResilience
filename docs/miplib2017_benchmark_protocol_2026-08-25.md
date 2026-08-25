# MIPLIB 2017 reproducible comparison protocol

## Rationale

Model/algorithm: censored single-thread MILP runs on the 240-instance MIPLIB
2017 benchmark set. A run is solved only when the backend proves the reference
status and every returned incumbent passes an original-model feasibility,
integrality, and objective audit.

Claim: under identical model files, limits, threads, and explicit random seeds,
the native branch-and-cut, HiGHS, and SCIP paths can be compared without
crediting fast incorrect returns or silently treating repeated runs with the
same seed as independent seeds.

Cost model: for `S` seeds and three solvers the experiment contains `720*S`
single-thread solves. With time limit `T`, the worst-case solver wall time is
`720*S*T`; model loading and the hard-timeout grace are additional costs.

Prediction: a fresh Release configuration builds and links 100% of the
benchmark and regression targets, without changing solver trajectories. Every
requested `(instance, solver, seed, repeat)` tuple appears exactly once in the
raw report. Solver performance has no predicted direction because the purpose
of the experiment is measurement, not optimization.

Assumptions: the input directory and `benchmark-v2.test` contain the same 240
official instances; all backends use one thread; the process watchdog enforces
the same wall limit; no other sustained CPU workload competes with the run.

References:

- Gleixner et al., "MIPLIB 2017", Mathematical Programming Computation 13
  (2021), Section 3 and Appendix A, DOI 10.1007/s12532-020-00194-3.
- Achterberg, "Constraint Integer Programming", PhD thesis (2007), Sections
  4.1 and 4.2, for branch-and-bound status and primal/dual bounds.
- SCIP Optimization Suite benchmark reports use shifted geometric means for
  time and nodes; this protocol uses shifts of 1 second and 100 nodes.

## Fixed metrics

- `solved`: a proven run whose incumbent passes the original-space audit and,
  when a reference is present, matches it; a proven reference infeasibility is
  also solved.
- `PAR-10`: solved runs use measured solve time and all other runs use `10*T`.
- `shifted_geomean_PAR10_ms`: `exp(mean(log(t_i + 1000))) - 1000`.
- `shifted_geomean_solved_nodes`: on solved runs with an available node count,
  `exp(mean(log(n_i + 100))) - 100`.
- Numerical audit: maximum original-space row, bound, integrality, and relative
  objective disagreement, plus the number of incumbent runs that fail the
  common `1e-5` threshold.
- Pairwise uncertainty: aggregate repeats within each instance first, then use
  10,000 instance-level bootstrap resamples for a 95% percentile interval.

Node counts are descriptive rather than a solver-independent work unit:
presolve, cut separation, LP iterations, and node definitions differ among
native, HiGHS, and SCIP. Pairwise node ratios therefore use only jointly solved
runs and are never substituted for PAR-10.

## Validation

```sh
cmake --preset macos-release --fresh
cmake --build --preset macos-release --target \
  miplib2017_benchmark test_branch_and_cut test_milp_solver \
  test_lp_solver test_netlib_regression
ctest --test-dir build/macos-release --output-on-failure \
  -R '^(test_branch_and_cut|test_milp_solver|test_lp_solver|test_netlib_regression)$'
python3 tools/run_miplib2017_audit.py \
  --benchmark tests/miplib2017_benchmark \
  --build-dir build/macos-release \
  --json reports/miplib2017.json --csv reports/miplib2017.csv -- \
  --data-dir tests/data/miplib2017/benchmark \
  --solu tests/data/miplib2017/miplib2017-v36.solu \
  --solvers native-highs-lp,highs-mip,scip-mip \
  --seeds 0,1,2 --repeat 1 --time-limit T --hard-timeout-grace 5
```

The final report records the exact command, build flags, executable and dataset
hashes, source state, commit, host, solver versions, and all raw results.

## Stability-gate mismatch and re-derivation

The clean-build validation exposed a reproducible allocator trap in
`test_milp_solver --rng-seed 1` before the benchmark campaign began. Address
Sanitizer localized the first invalid access to HiGHS `HFactor::btranU`, called
by `DirectHighsLpBasisOps::basis_inverse_row_sparse_entries`. The wrapper
allocated an `HVector` for six rows while its shared mutable `Highs` model had
already committed a seventh live-appended cut row.

The mismatch protocol gives the following diagnosis, in required order:

1. **Implementation infidelity: confirmed.** A successful live-append solve
   replaced `root_relax` and `root.basis_hint`, but left `root_sbasis` pointing
   at the pre-append wrapper. The wrapper's cached row count and matrix no
   longer described the shared HiGHS factorization.
2. **Machine/cost-model error: rejected as the root cause.** Redirection only
   changed whether the macOS allocator detected the already-invalid write; it
   did not create the dimensional inconsistency. ASan detects the same
   one-element heap overflow at the HiGHS basis API boundary.
3. **Assumption violation: confirmed.** The implementation assumed a basis
   wrapper remained immutable while its shared HiGHS owner was mutated by a
   live row append. The corrected invariant is that every basis solve requires
   `Highs::getNumRow() == BasisOps::row_count()`, and a committed append must
   publish the newly imported basis wrapper to every current basis owner.
4. **Theory error: rejected.** The algebraic operation remains a solve with the
   current square basis, `B^T y = e_i`; the defect was that the buffer and
   matrix represented an older basis dimension.

### Rationale for the stability repair

Model/algorithm: transactional live row insertion followed by dual-simplex
reoptimization. A successful transaction changes the basis from an
`m`-dimensional factorization to an `(m+k)`-dimensional factorization, so the
published `BasisOps` object must change with it. Basis inverse queries solve
`B x = r` or `B^T y = r` in exactly that current dimension.

Claim: after a successful live-append solve, publishing
`root_relax.simplex->basis.cached_sparse_basis` as `root_sbasis`, together with
a precondition guard on every direct HiGHS basis query, prevents stale-owner
memory access without changing any valid LP solve, pivot choice, objective, or
node decision.

Cost model: publishing the new shared pointer is `O(1)`. Each direct basis
query adds one `Highs::getNumRow()` comparison, also `O(1)`, outside the
factorization solve whose cost is at least `Omega(m)`. The predicted runtime
effect on the MILP test fleet is below 0.1%.

Prediction: the fixed-seed ASan reproducer changes from a deterministic
heap-buffer-overflow to 0 failures in 100 runs. Release mode changes from an
allocator trap to 0 failures in 100 redirected runs. For runs that never use a
stale wrapper, solver objectives, node counts, and pivot trajectories remain
identical.

Assumptions: HiGHS basis solve APIs size their input/output storage by the
current model row count; `build_native_simplex_state_from_highs` imports a new
wrapper after every successful live-append solve; rollback restores the old
HiGHS row count before the old wrapper becomes current again.

References:

- HiGHS basis inverse API and `HFactor::btranU` dimensional contract in the
  vendored HiGHS source used by commit `4a0b16a00daefcbe18d2c5648fe30f840ed77052`.
- Achterberg, "Constraint Integer Programming" (2007), Section 4.1, for the
  LP basis associated with each branch-and-cut relaxation.
- This document, "Stability-gate mismatch and re-derivation", for the ownership
  and dimension invariant introduced by the repair.

Validation is fixed before implementation:

```sh
# ASan build, fixed reproducer repeated 100 times
for i in {1..100}; do ./tests/test_milp_solver --rng-seed 1 >/dev/null; done

# Release build, fixed reproducer repeated 100 times
for i in {1..100}; do ./tests/test_milp_solver --rng-seed 1 >/dev/null; done

# Complete suite in both scheduling modes
ctest --test-dir build/macos-release --output-on-failure -j1
ctest --test-dir build/macos-release --output-on-failure -j4
```

Acceptance requires zero ASan findings, zero process failures in both 100-run
loops, and all CTest cases passing. A valid-path trajectory test, when one is
available for the affected root-cut case, must be bit-identical before and
after the guard; otherwise the repair is treated as an algorithmic change and
must pass the repository's full accuracy gates before benchmarking.

## Pilot mismatch: gap-certificate normalization

A 12-instance, seed-0 pilot with a 10-second limit completed 0, 1, and 1
instances for native, HiGHS, and SCIP under the initial reporting rule. The
same raw results showed that SCIP stopped `neos-3083819-nubu` at its configured
relative-gap limit (`3.25e-5 <= 1e-4`) but was not counted, while HiGHS called
its `9.99e-5` termination `Optimal`. HiGHS was then rejected because the
reference-objective comparison used `1e-5`, a stricter target than the
configured solve gap. Thus the initial solved counts compared backend status
vocabularies rather than a common mathematical stopping condition.

Mismatch diagnosis:

1. **Implementation infidelity: confirmed.** `SCIP_STATUS_GAPLIMIT` did not
   set the common `proven` flag, although the HiGHS equivalent did.
2. **Machine/cost-model error: rejected.** The discrepancy is deterministic
   metadata classification after each solve.
3. **Assumption violation: confirmed.** The reporting code assumed every
   backend uses `optimal` for a proof to the requested relative gap.
4. **Theory error: rejected.** A primal/dual interval satisfying the requested
   relative gap is the experiment's solver-independent completion criterion;
   exact equality with a known optimum is a stronger, separately reported
   audit.

### Rationale for the normalization repair

Model/algorithm: a run is solved to tolerance `g` when the backend certifies
infeasibility/unboundedness as appropriate, or returns an original-space
audited incumbent with a valid primal/dual relative gap at most `g`. For a
known optimal objective, the incumbent/reference disagreement is accepted for
the solved count at `max(1e-5, g)` and is always retained as a numerical-audit
quantity.

Claim: treating SCIP `GAPLIMIT` as a proof to the configured tolerance and
using the same tolerance for reference consistency makes equivalent HiGHS and
SCIP certificates receive the same solved classification. It does not excuse
large objective errors: for example, native `net12` at 255 versus the known
214 remains rejected at `g=1e-4`.

Cost model: the repair changes only `O(1)` post-solve comparisons and report
metadata, with predicted runtime impact below measurement resolution. On the
frozen pilot, the predicted solved counts change from native/HiGHS/SCIP
`0/1/1` to `0/2/2`; all raw times, nodes, incumbents, bounds, and violations
are produced by unchanged solve paths. Deterministic objectives, bounds, node
counts, and violations must match the frozen pilot; wall times may exhibit
ordinary run-to-run noise.

Assumptions: each backend's gap-limit status is based on a valid global dual
bound; the common original-space audit independently checks the returned
incumbent; `--gap` is finite and nonnegative.

References:

- Gleixner et al., "MIPLIB 2017" (2021), Section 3, for benchmark termination
  and performance comparison under a fixed gap tolerance.
- SCIP 10 status semantics, `SCIP_STATUS_GAPLIMIT`; HiGHS model-status and
  `mip_rel_gap` semantics in the vendored versions recorded by the report.
- This document, "Fixed metrics", for the common solved definition.

Validation is fixed as: rebuild the benchmark; rerun the exact 12-instance
pilot; require predicted solved counts `0/2/2`, unchanged per-run numerical
objectives, bounds, node counts, and violation values, and zero incumbent audit
failures; then rerun both complete CTest schedules. Any other change triggers a
new mismatch analysis before the full 240-instance experiment.

### Normalization validation mismatch

The first validation run measured solved counts `0/2/1`, versus the predicted
`0/2/2`. Investigation in the required order found:

1. **Implementation infidelity: confirmed.** SCIP `GAPLIMIT` correctly set the
   common `proven` flag after the repair, but `attach_validation` still entered
   known-optimum reference matching only when `result.optimal` was true.
   Consequently the normalized status could not set `reference_match` and was
   still excluded from `benchmark_solved`.
2. **Machine/cost-model error: rejected for solved classification.** This is a
   deterministic Boolean post-processing path.
3. **Assumption violation: confirmed for the node-count prediction.** Node
   counts on runs stopped by a wall-clock limit varied with scheduling (for
   example, HiGHS `cbs-cta` 227 to 241); they are not deterministic pivot-path
   gates. Completed, non-time-limited solver paths remain the valid trajectory
   comparison scope.
4. **Theory error: rejected.** The common gap-certificate definition is
   unchanged; every proof-to-tolerance with an incumbent must pass reference
   matching, whether the backend labels it `optimal` or `gap-limit`.

Corrected claim: known-optimum reference matching applies to every
proof-to-tolerance that returns an incumbent (`proven && has_solution`), while
infeasibility remains in its separate no-incumbent branch. The repeated pilot
must yield `0/2/2`, zero incumbent audit failures, and unchanged completed-run
objectives/bounds/violations. Node counts are compared only for runs that
complete before the wall limit; timeout-run node counts remain descriptive.

## Measured validation results

Source commit: `4a0b16a00daefcbe18d2c5648fe30f840ed77052` plus the
dirty-worktree changes recorded by the final audit manifest. Host: Apple M4
Max, 16 logical CPUs, 128 GiB RAM, macOS 26.5.2. The final clean Release
configuration used AppleClang 21.0.0 for C/C++ and GNU Fortran 16.2 for MUMPS.

- Stability repair: ASan fixed-seed loop passed 100/100, versus the predicted
  100/100; Release fixed-seed loop passed 100/100, versus 100/100. No ASan
  report or process failure occurred. The added check's isolated runtime cost
  was below the resolution of the test-level measurement, consistent with the
  predicted effect below 0.1%.
- Clean build: after cleaning `build/macos-release`, `cmake --preset
  macos-release --fresh` and `cmake --build --preset macos-release` reached
  100%. The final executable does not link the ASan runtime.
- Regression gates: both `ctest --test-dir build/macos-release
  --output-on-failure -j1` and the corresponding `-j4` run passed 18/18.
- Gap normalization: the repeated 12-instance pilot measured solved counts
  native/HiGHS/SCIP `0/2/2`, exactly matching the corrected prediction, with
  zero incumbent audit failures. Native `net12` remained rejected at audited
  objective 255 versus reference 214.

The pilot accumulated 369.4 solver seconds for 36 ten-second runs. Linear
extrapolation for the required 2,160-run matrix is approximately 6.2 hours at
`T=10 s` and 37 hours at `T=60 s`, before load and decompression variation.
Because the ten-second pilot solved only 0--2 of 12 instances per backend, it
is a smoke/stability experiment and must not be reported as a solver
state-of-the-art comparison. The formal campaign remains gated on an explicit
scientific time-limit choice.

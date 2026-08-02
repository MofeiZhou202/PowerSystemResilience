# Native IPM enhancement progress (2026-08-02)

This note records measured work derived from `native_ipm_audit_report.tex` and
`native_ipm_parallel_enhancement.tex`. It deliberately separates implemented
changes, verified results, invalid comparison data, and remaining work.

## Milestone 1: trustworthy results and CPU hot paths

Implemented:

- P1: acceptable convergence now requires primal feasibility, dual
  feasibility, and complementarity to all satisfy `tol_accept`. Returned
  multipliers are no longer replaced by zeros.
- P2: removed the frozen startup finite-difference Hessian and its `n`
  additional gradient evaluations. Hessian-free models now use the evolving
  sparse quasi-Newton state that was previously computed and discarded.
- P4/P5: removed filter `direct_accept`/`aggressive_direct` bypasses and fixed
  the switching condition so `filter_theta_min_scale` is effective.
- Filter line search: rejected trial points evaluate only objective and
  constraint values. Gradient and Jacobian callbacks are evaluated at accepted
  iterates, not at every backtrack.
- Filter dual step: the fraction-to-boundary dual step is fixed during primal
  backtracking instead of shrinking with the primal trial step.
- Filter history is retained across barrier reductions.
- P8: `use_inertia_correction` now selects the inertia-corrected or ordinary
  regularized condensed KKT path, including reused Gondzio/SOC right-hand
  sides. The obsolete `reduced_kkt_max_eq` field was removed.
- P11/result provenance: external Ipopt fallback is now explicit and disabled
  by default (`allow_external_fallback=false`). Native Merit fallback is
  reported as `NativeIPM[MeritFallback]`; Ipopt paths retain their explicit
  `NativeIPM[IpoptFallback]`/`NativeIPM[IpoptWarmStart]` names.
- P12: Merit convergence uses the requested complementarity tolerance without
  silently replacing it by `max(tol_complementarity, 10*tol_primal)`.
- T1.1/H2: Merit predictor, corrector, and Gondzio right-hand sides reuse one
  KKT factorization per iteration.
- Scaling correctness: nonlinear slacks are unscaled together with
  multipliers, and final KKT residuals are recomputed and certified against the
  original model before success is reported.
- `benchmark/opf_scale_probe.cpp` now constructs a genuinely feasible planted
  model, supports `native|ipopt|both`, disables hidden native fallback, aligns
  requested component tolerances, and prints actual solver identity plus all
  KKT components. Repeated runs are emitted as individual samples and `both`
  mode alternates solver order; this reduces fixed ordering bias but does not
  provide process isolation.
- The MUMPS comparison baseline was traced to an ABI-invalid build: vendored
  MUMPS 5.7.3 headers were paired with Homebrew Ipopt's private MUMPS dynamic
  libraries when `MIPSOLVERS_FORCE_BUILD_MUMPS=OFF`. `JOB=3` then reported
  success without changing the right-hand side. The hermetic vendored MUMPS
  build solves the same systems correctly.
- `MIPSOLVERS_FORCE_BUILD_MUMPS=OFF` is now rejected at configure time. A
  supported system-MUMPS mode needs an explicitly ABI-matched header and
  library discovery implementation; borrowing Ipopt's private libraries is
  not accepted.
- `MumpsSolver` now checks dimensions and sparsity-pattern reuse, rejects
  nonzero `INFOG(1)`/`INFOG(28)`, enables null-pivot detection, exposes
  negative-pivot/deficiency data, correctly restarts MUMPS analysis state, and
  certifies every returned solution with a scaled residual threshold of
  `1e-9`.
- Inertia-corrected augmented KKT systems use the symmetric MUMPS LDLT path
  when available. Tangent spaces of dimension at most 32 retain an independent
  reduced-space certificate; larger systems use factor negative pivots
  directly, and the two sources are cross-checked when both exist.
- Quasi-Newton secants now use Lagrangian-gradient differences with the new
  multipliers held fixed, including nonlinear constraint Jacobian curvature.
  The implementation is a sparse limited-memory SR1 approximation, not a
  complete L-BFGS implementation.
- A narrowly guarded centrality-only acceptance handles Newton steps whose
  primal displacement is at roundoff scale. It requires componentwise
  primal/dual non-worsening and a strict complete-KKT merit reduction with
  dual or complementarity progress; it does not reintroduce the removed
  general direct-accept bypass and it does not add a primal filter entry.
- At the minimum barrier, the filter driver now tightens its perturbed-KKT
  inner tolerance before giving up. Previously `S*mu-mu_bar <=
  kappa_epsilon*mu_bar` could pass while the user-visible `S*mu` remained just
  above `tol_complementarity`; the driver then stopped without taking the
  final Newton refinement and mislabeled the result as a maximum-iteration
  failure. A distinct minimum-barrier failure status is now used if the
  tightened solve still cannot meet the complete KKT test.
- An opt-in least-squares equality-dual initialization projects stationarity
  through a symmetric MUMPS KKT solve, accepts only finite multipliers below
  `constr_mult_init_max`, and requires a measured stationarity improvement.
  It is disabled by default because the model API does not identify nonlinear
  constraints whose multiplier-dependent curvature is unavailable. Explicit
  dual warm starts always take precedence, and pure quasi-Newton models defer
  this initialization.
- Slack-only warm starts now recompute their default inequality multipliers;
  previously changing `s` without also supplying `mu` silently destroyed the
  intended initial centrality relation.

## Verification

Scoped Release verification on Apple arm64, 2026-08-02:

| Check | Result |
|---|---|
| Native IPM (`test_ipm_solver`) | 23 cases, 154 assertions passed |
| full numerical stability suite, including hardened MUMPS | 16 cases, 409 assertions passed |
| conic IPM with hardened MUMPS | 17 cases, 213 assertions passed |
| hermetic Ipopt parameter stability | 6 cases, 441 assertions passed |

The normal `mipsolvers` target and all four scoped test targets built and linked
successfully before the last IPM run. During the final test-only edit,
concurrent MILP work changed separator signatures again: `bc_cuts.cpp` call
sites omitted the new `SeparatorStorageStats*` argument. The current full
target therefore fails in unrelated MILP code. The final Native IPM test
object was compiled and relinked against the already-current library archive;
no MILP file was modified or reverted as part of this work. This changing
build state is recorded rather than treating either the earlier successful
full build or the later unrelated failure as permanent.

The `build/macos-release/test_ipm_solver` executable is an older June artifact
and still dynamically links Homebrew Ipopt's MUMPS libraries; it is not
evidence for the results above. CMake's configured runtime output is
`tests/test_ipm_solver`, which links vendored MUMPS statically. An earlier
mixed-ABI numerical executable exited with signal-derived status 138 and is
excluded because its stale LP objects and new LP ABI made that binary invalid.
This was a build-integrity failure, not an IPM pass or failure.

The planted OPF-structure probe is an easy feasible-start case. It is not a
CUTEst-style benchmark and cannot establish general robustness or superiority.
Independent-process samples after the centrality acceptance and symmetric
MUMPS KKT changes were:

| buses | Native samples (ms) | Ipopt samples (ms) |
|---:|---|---|
| 4 | 1.316, 1.242, 1.134 | 1.716, 0.878, 0.870 |
| 25 | 1.538, 1.467, 1.458 | 1.152, 1.116, 1.104 |
| 100 | 3.231, 3.174, 3.364 | 2.393, 2.266, 2.116 |
| 400 | 12.447, 12.196, 12.162 | 6.255, 6.210, 6.581 |
| 1600 | 62.177, 60.972, 61.250 | 228.726, 33.375, 25.784 |

The 228.726 ms Ipopt observation is retained as measured. Three samples are
insufficient to characterize either distribution. Native still needs seven
barrier iterations while Ipopt needs one on this planted feasible-start case,
so even the favorable 1600-bus samples do not justify a general speed claim.

At 1600 buses, profiling attributed the main Native improvement to the linear
backend: the previous generic unsymmetric factorization took roughly
0.43--0.49 seconds per factor, while symmetric MUMPS LDLT took roughly
0.0074--0.0082 seconds after initial analysis. End-to-end Native time dropped
from roughly 2.77 seconds to roughly 61 ms on this probe. These are probe-local
measurements, not a claimed universal speedup.

### Multiplier-initialization experiment

The first implementation used unrestricted Eigen `SparseQR`. It reduced the
1600-bus probe from seven to six iterations but increased a single Native run
from the roughly 61 ms baseline to **6002.225 ms**. That implementation was
rejected and removed; the result is retained here because reporting only the
iteration reduction would be misleading.

The replacement uses the hardened symmetric MUMPS projection KKT. Separate
process samples at 1600 buses were:

| configuration | iterations | samples (ms) |
|---|---:|---|
| explicit `ls-duals` | 6 | 64.752, 63.563, 65.250 |
| explicit `no-ls-duals` | 7 | 80.800, 65.631, 66.833 |

The enabled median is 64.752 ms and the disabled median is 66.833 ms, but six
total samples and the 80.800 ms observation do not establish a stable speedup.
The feature therefore remains opt-in. The probe prints `ls_duals`, run index,
order position, actual solver identity, all KKT components, and every timing
sample; its default is `no-ls-duals`, matching `IPMOptions`, and `both` mode
alternates order but still shares one process.

## Not yet complete

No claim of broadly outperforming Ipopt is supported yet. Major remaining
items include:

1. Restore a clean full build after the concurrent MILP separator API change
   is internally consistent, then rerun the full repository suite.
2. Extend the new regression matrix beyond the current infeasible-start,
   nonconvex, rank-deficient, and inconsistent-equality cases, especially to
   nonlinear degeneracy and aggressive barrier-reduction cases.
3. Implement and validate a safeguarded adaptive barrier oracle. The opt-in
   multiplier initialization only closes the observed seven-versus-one gap to
   six-versus-one; it is not a substitute for probing/quality-function logic.
4. Add native infeasibility detection/certificates and restoration for pure
   inequality problems.
5. Add scaled Ipopt-style termination, derivative
   checking, and variable scaling.
6. Extend the symmetric-indefinite backend to real multi-threaded execution,
   then profile assembly, factorization, solves, evaluations, and line search
   separately with 1/2/4/8/16-thread sweeps.
7. Run a pinned benchmark matrix on CUTEst, including well-conditioned,
   nonconvex, degenerate, infeasible, and large sparse cases. Report failures,
   KKT residuals, iterations, evaluations, and distributions over repeated
   timings; never aggregate fallback results under the native label.

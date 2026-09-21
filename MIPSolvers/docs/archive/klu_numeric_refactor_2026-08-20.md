# KLU numeric refactor for fixed-pattern LE/NLE (2026-08-20)

## 1. Scope and prediction

This change ports the KLU numeric-refactor capability from `main` commit
`3bf1e667`, then constrains it to the cached power-flow Newton path. It does
not alter the one-shot LE adapter or the generic NLE adapters that rebuild or
reanalyze their Jacobian each iteration.

For a fixed compressed-column pattern, a full KLU numeric factorization pays
for numerical pivot selection and factor storage construction. `klu_refactor`
retains the ordering, block structure, factor storage, and pivot order, and
only updates numeric values. The pattern guard is O(nnz); sparse LU numeric
work is superlinear in the eliminated fronts. The prediction fixed before
implementation is therefore:

- one-shot LE: 0% change;
- fixed-pattern NLE factorization time: 10%-35% lower;
- fixed-pattern NLE end-to-end time: 5%-20% lower when factorization is a
  material part of runtime;
- no accepted-solution or residual change within existing accuracy gates.

The assumptions are at least two successful Newton factorizations, stable
Jacobian structure, and a first factorization whose pivot order remains
numerically viable for later values.

## 2. Algorithmic contract

SuiteSparse KLU defines `klu_refactor` as factorization with the same ordering
as `klu_factor`. Its implementation states two non-negotiable preconditions:

1. it cannot perform numerical pivoting;
2. input `Ap` and `Ai` must be identical to those supplied to `klu_factor`.

References: Davis and Palamadai Natarajan, "Algorithm 907: KLU, A Direct
Sparse Solver for Circuit Simulation Problems," ACM TOMS 37(3), 2010;
SuiteSparse 7.12.2 `KLU/Source/klu_refactor.c`, lines 11-15, and
`KLU/Include/klu.h`, the `klu_refactor` API contract.

Consequently, equality of dimensions and nonzero count is insufficient. The
production wrapper stores the successful factorization's complete compressed
column pointers and row indices, and compares both arrays exactly before
calling KLU. Uncompressed inputs are ineligible because an implicit temporary
could change the storage representation whose identity KLU requires.

## 3. Transaction and fallback

The cached Newton solver maintains a `numeric_factor_ready` state. The first
matrix always uses the existing symbolic analysis plus full `factorize`.
Later matrices may use `refactorize` only when the backend advertises support
and a successful numeric factor exists.

Refactor is speculative. Pattern rejection, singularity, or fixed-pivot
failure rolls back to:

1. `analyze_pattern(current_matrix)`;
2. full pivoting `factorize(current_matrix)`;
3. publish the numeric-ready state only on success.

This rollback is also required when Newton temporarily switches from `J` to a
regularized matrix or `J^T J` during recovery. It prevents a symbolic object
for one structure from being consumed by another. The fast path never changes
the acceptance, line-search, residual, or convergence rules.

## 4. Validation protocol

Fixed before measurement:

- direct KLU tests: reject before first factorization, accept same-pattern new
  values, reject changed pattern, recover through full analysis, reject a
  singular refactor, then recover a well-posed factorization;
- Windows Release full suite: all 19 CTest targets pass;
- external installed-package consumer still configures, links, and returns
  objective `9.000000`;
- installed CMake/header files contain no build/user paths and PE dependencies
  contain no MKL/OpenMP/ZLIB/SCIP/Ipopt DLL;
- macOS/Linux performance must be reported with command, build flags, commit,
  factorization time, and total NLE time before widening the route.

If the speedup has the wrong sign or differs from the prediction by more than
about 50%, investigate in order: fast-path hit rate and pattern fidelity,
factorization share/cost model, fixed-pivot numerical failures, then the
underlying prediction. Record the mismatch here before further code changes.

## 5. Measurement record

Windows measurement environment:

- source base: `571fe73fedb803d6aa5a4ea8fb6917c0ef9bca9b`, with the
  uncommitted hardened port of `main` commit `3bf1e667`;
- compiler/build: MSVC 19.44.35207, Ninja Multi-Config, Release `/MD`, static
  sequential oneMKL, vendored SuiteSparse 7.12.2, OpenMP disabled;
- command: `tests/Release/klu_refactor_benchmark.exe`;
- workload: `n=4000`, `nnz=19984`, 80 updates per trial, seven-trial median,
  three independent process runs.

The three process runs reported factor/refactor reductions of 75.85%, 75.56%,
and 74.31% (4.141x, 4.092x, and 3.892x speedups). The middle run by speedup is
the recorded result: full factorization 73.828 ms total (0.922849 ms/call),
numeric refactor 18.041 ms total (0.225515 ms/call), or a 75.56% reduction.
Both relative residuals were `1.421e-15`; the two final solutions were
identical at the reported precision.

### 5.1 Prediction mismatch and re-derivation

The measured 75.56% reduction exceeds the predicted 10%-35% interval by more
than half the predicted effect, so the mismatch protocol was applied before
further code changes:

1. Implementation fidelity: the refactor call succeeded for every update,
   the exact CSC guard ran inside the timed call, both solves passed, and the
   solution delta was zero. No rejection/fallback contaminated the fast path.
2. Machine/cost model: the original model counted the O(nnz) pattern check and
   sparse triangular numeric work, but did not separately price pivot search,
   numeric-object construction, and allocation performed by `klu_factor` and
   retained by `klu_refactor`. On this narrow-band matrix, arithmetic is cheap
   enough that those omitted fixed/metadata costs dominate.
3. Assumptions: the pattern is fixed, the matrix remains strictly diagonally
   dominant, and the original pivot order remains viable. No assumption was
   violated by this controlled workload.
4. Theory: the predicted direction remains supported, but a single percentage
   interval cannot cover both narrow structured matrices and fill-heavy power
   flow Jacobians.

The revised prediction for this controlled narrow-band microbenchmark is a
55%-80% factorization-time reduction. The original application-level
prediction remains deliberately unchanged until real fixed-pattern NLE cases
measure fast-path hit rate, factorization share, and end-to-end time. This
prevents the favorable synthetic result from being generalized to production
workloads without evidence.

Windows release validation completed under the same build configuration:

- directed KLU lifecycle test: 15 assertions in two cases passed;
- `ctest --test-dir build/windows-production-portable -C Release
  --output-on-failure`: 19/19 targets passed in 158.85 seconds;
- an external consumer copied to a fresh temporary source/build tree,
  configured with only the installed SDK in `CMAKE_PREFIX_PATH`, compiled,
  linked, and returned objective `9.000000`;
- installed headers and CMake files contained no source/build/user path;
- `dumpbin /DEPENDENTS` reported only Windows system DLLs and the MSVC runtime,
  with no MKL/OpenMP/ZLIB/SCIP/Ipopt DLL;
- installed static oneMKL payload was 638,380,710 bytes and all nine staged
  license files were present.

The macOS/Linux performance claim remains unaccepted until reproduced on those
platforms under the protocol in section 4.

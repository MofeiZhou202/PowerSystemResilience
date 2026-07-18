# Code Quality Evaluation — `include/` + `src/`

**Date:** 2026-07-18
**Scope:** `include/mipsolvers/**`, `src/**` (~141k lines, 240 files)
**Focus:** numerical stability and scalability toward million-dimension problems with MKL Pardiso-class performance
**Method:** static review of the sparse LU core, IPM/KKT/Newton kernels, dual simplex/PDLP/MILP, and cross-module architecture. All line numbers refer to the working tree at evaluation time.

---

## 1. Executive Summary

The codebase has **strong algorithmic skeletons but engineering that does not yet meet them**:

- The NLP Filter IPM is architecturally close to Ipopt (filter line search, Wächter–Biegler inertia-correction schedule, SOC, restoration, problem scaling).
- The sparse dual simplex implements modern machinery: exact DSE (Goldfarb–Reid), BFRT bound-flipping ratio test, Harris two-pass, EXPAND anti-degeneracy, adaptive refactorization triggers.
- The IPM-LP kernel is the highest-quality module (precomputed scatter maps, symbolic-analysis reuse, banded/sparse/Accelerate branches, NEON kernels).

However, for the stated goal (million-dimension matrices, Pardiso-class performance) the current code is not "slow" — **it would fail or crash outright**. Four blocking issues:

1. **32-bit indexing everywhere** — silent overflow once factor nnz exceeds 2³¹.
2. **Absolute-threshold dense fallbacks** — TB-scale dense allocations on million-dim sparse problems.
3. **Zero parallelism and no BLAS** in all numerical kernels; default backend is serial `Eigen::SparseLU`.
4. **Deep-copy chains** — 3–5 full-model copies per solve, plus per-node full standard-form copies in B&C.

### Readiness scores (million-dimension)

| Module | Score | Main gap |
|---|---|---|
| Sparse LU core (`kernel/linear_algebra/`) | 3/10 | int32 overflow, no parallelism, unbounded FT-update growth |
| IPM / KKT / Newton | 4/10 | TB-scale dense fallback, no iterative refinement on LP path, single-threaded |
| Dual simplex / PDLP / MILP | 4/10 | LP kernel 8/10 but MILP tree 2/10 (O(n) node storage) |
| Architecture / memory model | 3/10 | copy chains, per-node copies, dead refactored stack |

---

## 2. Fatal Issues for Million-Dimension Operation

### 2.1 32-bit index ceiling (silent overflow)

All sparse matrices use Eigen's default `StorageIndex=int`; UMFPACK is called via `umfpack_di_*` (int) instead of `umfpack_dl_*` (long):

- `src/engine/kernel/linear_algebra/sparse_lu_factor.cpp:65,75`
- `src/engine/kernel/lp_kernel/dual_simplex.cpp:3807,3813,3878`
- vendored `src/engine/kernel/linear_algebra/highs_factor/HConfig.h` does not define `HIGHSINT64`
- `src/engine/kernel/kkt/kkt_system.cpp:75` — `static_cast<int>(nnz)`

Dimension 10⁶ itself is fine, but L/U factor nnz beyond 2³¹ (plausible for million-dim LP bases) overflows `int` counters silently. No overflow asserts anywhere.

### 2.2 Dense fallback is a TB-scale memory bomb (correctness bug)

`src/engine/kernel/ipm/ipm_lp_solver.cpp:523-531`: when the scatter estimate `Σ_j nz_j² > 4M`, the solver switches to a dense path allocating an m×n dense matrix. At n=10⁶ with only 10 nonzeros/column, `Σnz² ≈ 10⁸ ≫ 4M` → attempts a TB-scale allocation. The cached path for B&C node LPs (`prepare_for_node_solves`, `:1489-1539`) has **no scatter cap at all** — a single dense column (nz=10⁵) causes `reserve(10¹⁰)` → `bad_alloc`. The criterion must be relative density / memory-feasibility, not an absolute scatter value.

### 2.3 Zero parallelism, zero BLAS

All numerical kernels (LU factorization, triangular solves, KKT assembly, IPM main loop) are serial scalar loops. No `#pragma omp`, no `std::thread` in any kernel; no `dgemm`/`dtrsv`; CMake has no `find_package(OpenMP)`. The default linear-solver backend is `Eigen::SparseLU` (`include/mipsolvers/engine/kernel/linear_algebra/linear_solver.hpp:21-32`) — serial, no BLAS, an order of magnitude slower than Pardiso on factorization alone. (Only exception: Apple Accelerate sparse Cholesky + `cblas_dsyrk` on the IPM-LP macOS path.)

### 2.4 Deep-copy chains (memory-bandwidth death)

Copies of the full model per LP solve:

- Engine API layer, 3 copies: `src/engine/api/solver.cpp:190,214-216` (`normalize_problem` takes the variant by value)
- B&C entry, 2 more: `branch_and_cut.cpp:1642,1651`
- **Every B&C node**: full `StandardFormLP` copy (`:23143-23168`), plus rebuild + Ruiz rescale when local cuts exist (`:23150-23153`)
- `Node` is a heavy value object: 4 full-length `Eigen::VectorXd` + ~15 `std::vector` members (`include/mipsolvers/engine/detail/bc_types.hpp:330-378`); n=10⁶ with 10k open nodes ≈ **160 GB**

The incremental design already exists — `TreeNode` storing only `bound_changes` (`bc_types.hpp:147-166`) — but is **never used** by the legacy code.

---

## 3. Numerical Stability Defects

### 3.1 Correctness bugs (fixed in Phase 0 — see §7)

1. **Banded Cholesky regularization retry factorizes a garbage matrix** — `ipm_lp_solver.cpp:1043-1048` (non-cached) and `:2062-2065` (cached): on retry, `dyn_reg` is added to the diagonal of the *already partially factorized* `band_work` buffer without refilling from the pristine matrix. The factorization "succeeds" but is mathematically wrong; the LP path has no step-acceptance check to catch it. `fill_and_factor_banded()`'s return value is discarded.
2. **`mu_aff/mu` 0/0 → NaN propagates to the returned solution** — `ipm_lp_solver.cpp:1135,2141`: when `n_compl==0` (all variables unbounded), `mu=0` → `0/0=NaN` → NaN pollutes the rhs; the LP loop has no `allFinite` guard and returns a NaN "solution" after MaxIter.
3. **`HFactorBackend::needs_refactorise()` is always false** — the wrapper uses `kUpdateMethodFt` (`hfactor_backend.cpp:145`), but `HFactor::update` only sets `*hint` for PF/MPF (`HFactor.cpp:527-530`); the FT branch's hint code is commented out (`HFactor.cpp:2412-2414`). Fill blow-up never triggers refactorization.
4. **Four unchecked `umfpack_di_wsolve` return values** — `dual_simplex.cpp:3878,3911,3964,4002`: a failed solve silently uses garbage results.

### 3.2 Missing stability mechanisms

- **Iterative refinement explicitly disabled**: `dual_simplex.cpp:3480` sets `Control_[UMFPACK_IRSTEP]=0`, while the FT update chain (up to 200 updates) accumulates error with zero correction. KKT path has ≤2 IR steps (`kkt_system.cpp:144-155`) but the LP IPM path has none.
- **FT update without multiplier-growth control**: `sparse_lu_factor.cpp:693-724` — `h_diag ≥ 1e-14` (absolute magic number) is accepted, multipliers can reach ~10¹³ unchecked; growth monitoring is a weak proxy. Each update also **rebuilds U's CCS+CSR in O(nnz(U))** (`:785-856`) where classic FT is O(nnz(spike)).
- **LP IPM has no scaling at all**: the header advertises "Ruiz + Pock-Chambolle equilibration" (`ipm_lp_solver.hpp:21`) but `ruiz_rounds` is **never referenced** — a dead option, along with `max_correctors`, `presolve`, `crossover` (false documentation).
- **NLP scaling computation is O(m·nnz)**: `ipm_scaling.cpp:10-17` materializes `M.row(i)` per row of a CSC matrix; a single O(nnz) column sweep would do. At million-dim the (default-on) scaling phase dies first.
- **Merit-path "acceptable" termination too loose**: `ipm_solver.cpp:2183-2194` — `tol_accept` defaults to **1e-2**, checks primal feasibility only, and returns constraint duals filled with zeros as "success". Complementarity tolerance is silently relaxed 10× (`:1893`).
- **Filter switching condition neutralized**: `ipm_solver.cpp:1177-1181` — `filter_s_phi` is always 2.3>0, making the Wächter–Biegler `θ_k ≤ θ_min` check tautological; f-type steps can be accepted at arbitrarily large infeasibility.
- **No infeasibility/divergence detection** in LP IPM (only MaxIter); NLP IPM has no native infeasibility certificate.
- **Inertia certificate via dense tangent space**: `kkt_system.cpp:462` builds `MatrixXd::Zero(n, nullity)` — at n=10⁶, nullity=128 this is a 1 GB temporary, rebuilt every iteration; proper practice reads inertia from the factorization (MA57/Pardiso-style).
- **`Jhᵀ·diag(μ/s)·Jh` explicitly formed** (`ipm_solver.cpp:960-961`): squares the condition number and fill; Ipopt keeps the diagonal `-S·Z⁻¹` block instead.
- File title claims Forrest–Tomlin but production runs PFI: `dual_simplex.cpp:2` vs `dual_simplex.hpp:76-78` and `MIPSOLVERS_ENABLE_FACTOR_BACKEND_B 0` (`dual_simplex.cpp:98-100`).
- Harris tolerance comment off by 100× (`dual_simplex.cpp:5457`, claims ~1e-4, actually 1e-6).

### 3.3 Gap analysis vs MKL Pardiso

| Pardiso capability | Status here |
|---|---|
| Supernodal + BLAS3 dense kernels | ❌ scalar sparse loops |
| METIS / nested-dissection ordering | ❌ only COLAMD / Markowitz / AMD |
| OpenMP parallel factorize/solve | ❌ fully serial |
| Matching (MC64-style) + scaling | ❌ row-sum scaling only |
| Iterative refinement | ❌ disabled (`dual_simplex.cpp:3480`) |
| ILP64 indexing | ❌ all int32 |
| Symmetric-indefinite Bunch–Kaufman (augmented systems) | ❌ normal equations + LDLᵀ only, with explicit `JhᵀDJh` |
| Symbolic/numeric factorization separation & reuse | ✅ present (UMFPACK and Accelerate paths) |
| **Pardiso wrapper** | ⚠️ **already exists** (`linear_solver.hpp:82-97`, `cmake/Dependencies.cmake:307-417`) but Linux/Windows-only and off by default |

---

## 4. Architecture-Level Issues

1. **35,317-line single file / 33,650-line single function**: `branch_and_cut.cpp`'s `branch_and_cut_lp` (`:1568→35222`), containing 383 lambdas and **203 environment-variable reads** (128 `std::getenv`). The same input takes different algorithmic paths depending on the environment — unauditable, irreproducible, un-CI-able.
2. **Dead parallel stack**: carefully redesigned components (`parallel/parallel_search.cpp`, `parallel/shared_state.cpp`, `search/sequential_search.cpp`, `root/root_solve.cpp`, `milp_presolve.cpp`, ~7k lines, well documented) have **no production caller** — `api.cpp:8` calls `solve_milp_bc_legacy_core` directly. Any B&C improvement must first decide which stack to modify.
3. **Node LU factors dropped on enqueue** (`bc_legacy_helpers.cpp:227-230`) — every dequeued node pays a fresh sparse LU before dual reoptimization (a correct thread-safety trade-off, but high-end solvers keep serialized/deep-copied factors).
4. **CSC+CSR dual residency** (`dual_simplex.hpp:271-272`, `pdlp_solver.cpp:252-269`) — ~2.4 GB redundant at 10⁸ nnz; SCUC holds a dense PTDF `Eigen::MatrixXd` (`src/scuc/scuc.cpp:861`).
5. **Near-zero test coverage of numerical cores**: dual simplex has four 2–3-variable micro-cases; PDLP has none; FT updates, dynamic-regularization retries, NaN guards, and large-scale regressions are all uncovered. None of the §3.1 bugs could be caught by the current suite.
6. Miscellaneous: `-march=native` defaults OFF (`CMakeLists.txt:29`); `ThreadPool` is a basic single-queue design with per-task heap futures; `NativeIPMLPAdapter`'s `mutable` caches are unsynchronized (contractual data-race landmine, `ipm_lp_solver.hpp:95-99`); `.DS_Store` files checked into `include/` and `src/`; stale `HACDCPF_` naming throughout; two duplicated vendored `pdqsort.h` copies; `dual_simplex.cpp:2825` embeds a ~600-line `NativeLU` that duplicates `SparseLUFactor::extract` near-verbatim.

### PDLP status

Ruiz equilibration, Pock–Chambolle preconditioning, primal-weight balancing, adaptive restarts, ergodic averaging are present, but: no Malitsky–Pock linesearch, no infeasibility certificates, restart/convergence metric mixes absolute ∞-norm violations with a relative gap (`pdlp_solver.cpp:455-456`), infinite gap treated as "satisfied" (`:593`), zero tests, not wired into B&C.

---

## 5. Roadmap to Million-Dimension, Pardiso-Class Performance

### Phase 0 — Correctness (the five Phase-0 fixes below; status tracked in §7)

1. Banded Cholesky regularization retry: refill from pristine storage before perturbing (`ipm_lp_solver.cpp:1043,2062`).
2. LP IPM `allFinite` guards + `n_compl==0` protection (`:1135,2141`).
3. Dense-switch criterion → memory-feasibility gate; add the same cap to the cached path (`:523-531,1489-1539`).
4. Restore FT fill-based refactorization hint (`HFactor.cpp:2412-2414`).
5. Check the four `umfpack_di_wsolve` return values (`dual_simplex.cpp:3878,3911,3964,4002`).

### Phase 1 — Scale unlock

6. Template/upgrade index types to int64 (`umfpack_dl_*`, `HIGHSINT64`, Eigen `StorageIndex=int64_t`) + nnz overflow asserts.
7. Eliminate copy chains: pass problems by const reference / immutable sharing; switch B&C nodes to the already-designed incremental `TreeNode`.
8. `row_infty_norm` → single O(nnz) column sweep; implement real Ruiz scaling in LP IPM.
9. Re-enable UMFPACK iterative refinement; add IR to the normal-equations path.

### Phase 2 — Performance parity (**integrate, don't rewrite**)

10. The Pardiso wrapper already exists — promote backend priority to Pardiso > CHOLMOD (new) > Accelerate > Eigen, default-on across platforms with runtime fallback. Do not write another LU.
11. Generalize the IPM-LP "analyze once, factorize many" contract (scatter maps + `SparseRefactor`) to the KKT/NLP paths — KKT currently reassembles from triplets every iteration (`kkt_system.cpp:85-118`).
12. Augmented system with diagonal `-S·Z⁻¹` block instead of explicit `JhᵀDJh`.
13. Introduce OpenMP: KKT assembly, SpMV, node-level B&C parallelism (wire up the dead `parallel/` components).

### Phase 3 — Engineering governance

14. Split `branch_and_cut_lp` along the existing `root/search/parallel` boundaries; converge the 203 env vars into an explicit options struct.
15. Build a NETLIB-subset regression suite + large-scale benchmarks — the only way to validate everything above.

---

## 6. Positive Highlights (worth keeping)

- Exact DSE with Goldfarb–Reid updates, BFRT, EXPAND, multi-signal adaptive refactorization (`dual_simplex.cpp:4327-4360`) — the most rigorous numerical-management code in the tree.
- NLP Filter IPM: Wächter–Biegler δ_W/δ_C schedule, SOC, Ipopt-style restoration, gradient scaling, symbolic-analysis caching shared across multiple RHS.
- IPM-LP kernel: precomputed scatter maps, symbolic reuse, banded/sparse/Accelerate branches, NEON kernels — the template every other kernel should follow.
- Super-sparse triangular solves (DFS reach + density gating) — the right direction for million-dim simplex; Pardiso doesn't even offer this.
- Healthy exception policy: only 4 `throw`s in the engine; solve paths return status codes.

---

## 7. Phase-0 Fix Log

| # | Bug | File | Status |
|---|---|---|---|
| 1 | Banded Cholesky reg-retry factorizes garbage | `ipm_lp_solver.cpp` | ✅ fixed — retry now restores the pristine matrix from `band_storage` / `band_storage_local` into `band_work` before perturbing the diagonal (both paths) |
| 2 | `mu_aff/mu` 0/0 NaN propagation | `ipm_lp_solver.cpp` | ✅ fixed — `sigma = 0` (pure affine step) when `mu == 0`; plus a per-iteration finiteness guard on `dx`/`dy` that aborts with status `NumericalError` instead of returning a NaN solution (both paths) |
| 3 | Absolute dense-switch criterion / cached path | `ipm_lp_solver.cpp` | ✅ fixed — dense path gated by a 256 MB memory budget (`kDenseMaxBytes`); scatter maps capped at 10⁸ entries (`kMaxScatterEntries`): cached path skips caching, non-cached path fails with status `ProblemTooLarge`; banded reserves use the tight `Σ nz·min(nz,bw+1)` bound instead of `Σ nz²` (both paths) |
| 4 | `needs_refactorise()` always false (FT hint) | `HFactor.cpp`, `hfactor_backend.*` | ✅ fixed — fill trigger `u_total_x > u_merit_x && pf_pivot_index.size() > 100` evaluated in `HFactor::update` right after `updateFT` (upstream semantics); `HFactorBackend` zero-initializes `hint` per update |
| 5 | Unchecked `umfpack_di_wsolve` returns | `dual_simplex.cpp` | ✅ fixed — all four sites check for `UMFPACK_OK` and poison the result with NaN so downstream `isfinite` guards abort |

**Validation:** `test_ipm_solver` (75 assertions / 9 cases), `test_dual_simplex` (11/4), `test_lp_solver` (36/5), `test_milp_solver` (70/6), `test_engine_api` (92/19) — all passing on macOS Release after the fixes.

---

## 8. Phase-1 Log

Scope (agreed): items 8 and 9 in full, with numerical tests; item 6 as loud
overflow guards; item 7 as API-layer copy elimination. Full int64 migration
and the B&C `TreeNode` rewrite remain deferred — they are ABI-wide / legacy-
rewrite efforts of their own, not single changesets.

| # | Item | Status |
|---|---|---|
| 8a | `row_infty_norm` O(m·nnz) → O(nnz) | ✅ done — `compute_scaling_factors` now computes all row ∞-norms in a single column sweep (`ipm_scaling.cpp`); bit-identical results |
| 8b | Real Ruiz scaling in LP IPM | ✅ done — `ruiz_rounds` (default 10) now performs alternating row/column ∞-norm equilibration of `[A; Aeq]` in both `solve_lp` and the cached node-LP path; cost/RHS/bounds scaled at setup, primal/dual/box-multiplier outputs unscaled (`x = Dc·x̂`, `y = Dr·ŷ`, `z = ẑ/Dc`). Slacks keep coefficient 1, so all slack logic is untouched. One implementation bug was caught by the new tests: row factors were accumulated per nonzero instead of per row (dr squared per round) — fixed before merge |
| 9a | Re-enable UMFPACK iterative refinement | ✅ done — `Control_[UMFPACK_IRSTEP] = 2` in `SparseBasis` (`dual_simplex.cpp`); UMFPACK early-stops, so the extra SpMV + triangular solve is only paid when the residual demands it |
| 9b | IR on the normal-equations path | ✅ done — conditional refinement in both `solve_normal` lambdas: residual `r = rhs − N·dy` against the pristine matrix (new `banded_sym_matvec` for the banded path, `selfadjointView<Lower>` for sparse/dense), ≤ 2 correction solves with the existing factorization when `‖r‖∞ > 1e-12·max(1, ‖rhs‖∞)` |
| 6 | int64 index migration | ⚠️ guards only — loud failure past the int32 ceiling at the doc-cited points (`kkt_system.cpp` dim/nnz truncation, `sparse_lu_factor.cpp` negative lnz/unz after `get_lunz`, `dual_simplex.cpp` `B_.nonZeros()` before `umfpack_di_symbolic`). Full migration deferred |
| 7 | Copy chains | ⚠️ partial — engine API layer down to 1 copy: `SolverEngine::solve` takes the variant by value and moves through `normalize_problem`; `solve_lp/qp/milp/minlp` move their normalized model into the variant (3 → 1 full-model copies). B&C-side copies and `TreeNode` migration deferred |

**Numerical tests** — new `tests/test_numerical_stability.cpp` (332 assertions / 6 cases):
ill-scaled LP rescued by Ruiz (entries 1e⁻¹⁰..1e¹⁰, exact optimum recovered);
`compute_scaling_factors` row-norm regression; near-parallel equality LP
(cond(N) ~ 4e10) solved accurately on the banded path; sparse-path IR sanity;
ill-conditioned equality LP through dual simplex; cached node LP ≡ fresh solve
with scaling on (primal, dual, box multipliers).

**Validation:** full unit tier green — `ctest -L unit`: 10/10 suites pass
(engine_api, lp, milp, ipm, dual_simplex, problem_validation, adapter_registry,
presolve, l2o_trace, numerical_stability). `test_ipopt_parameter_stability`
(integration tier) has one assertion failure that reproduces on the pristine
tree (verified via stash-check) — pre-existing, unrelated to these changes.

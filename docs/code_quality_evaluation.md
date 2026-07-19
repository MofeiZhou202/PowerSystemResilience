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

---

## 9. LA Backend Log — vendored CHOLMOD (roadmap item 10, partial)

**Adopted open-source LA package: CHOLMOD 5.3.4** (SuiteSparse 7.12.2,
supernodal BLAS-3 sparse Cholesky) as the portable SPD backend, replacing the
serial `Eigen::SimplicialLDLT` fallback.

- **Vendored for offline builds** — the needed SuiteSparse subset
  (`SuiteSparse_config`, `AMD`, `CAMD`, `COLAMD`, `CCOLAMD`, `CHOLMOD`) is
  committed in-tree under `suitesparse/` (~24 MB) and compiled by
  `cmake/BuildCHOLMOD.cmake` into the static `cholmod_vendored` target
  (sources inlined, no FetchContent/network at configure, build, or deploy
  time; mirrors the `highs/` `scip/` `ipopt/` convention). BLAS for the
  supernodal kernels: Accelerate on macOS, system BLAS/LAPACK elsewhere
  (graceful degradation to simplicial if absent). Exported in
  `mipsolversTargets`; system `libcholmod` is skipped to avoid symbol
  collisions.
- **Wrapper** — `cholmod_ldlt.{hpp,cpp}` (RAII over the int64 `cholmod_l_*`
  API): analyze once per sparsity pattern, zero-copy factorize per iteration
  (values alias `N_sparse`/`N_local`), preallocated solve workspace.
- **Integration** — LP IPM sparse normal-equations path, both `solve_lp` and
  the cached node-LP path, including init least-squares and dynamic-reg
  retries; CHOLMOD solve failures poison with NaN into the Phase-0 finiteness
  guard.
- **Backend priority (measured, not assumed):** Accelerate (macOS) > CHOLMOD >
  Eigen. On a 3000×63000 sparse LP (N: 3000², ~55k nnz), 19 IPM iterations:
  Accelerate factor time **0.45–0.50 s** vs CHOLMOD **1.29–1.61 s** — Apple's
  sparse Cholesky is ~2.7× faster on Apple Silicon even with both on
  Accelerate BLAS, so macOS keeps Accelerate; CHOLMOD is the primary backend
  where Accelerate does not exist (Linux deployment), where the previous
  fallback was serial Eigen.

**Tests:** `test_numerical_stability` gains a direct `CholmodLDLT` case
(factorize/solve vs analytic, analyze-once/factorize-many refresh, not-PD
rejection). Full unit tier green: 10/10 suites.

**Follow-ups (explored, queued):** MUMPS for the indefinite KKT path
(vendored `cmake/BuildMUMPS.cmake` already exists for Ipopt); UMFPACK
`di`→`dl`; `HIGHSINT64`; B&C copy elimination (entry copy + per-node SF
workspace + Node slimming — full file:line map available).

---

## 10. Phase-1 Closure, Phase-2/3 Log, and Theoretical Notes

Derivations for the numerical choices in this section live in
**`docs/numerical_methods.md`** (one-sided rows, Ruiz scaling and its failure
modes, iterative refinement, analyze-once/factorize-many, condensed vs
augmented Newton systems, index width, backend selection, memory-bandwidth
model of the copy chains).

### Item 6 — int64 indexing (closed)

| Slice | Status |
|---|---|
| UMFPACK `di`→`dl` | ✅ `dual_simplex.cpp` (SparseBasis: `Ap_/Ai_`/`wsolve_Wi_` → `int64_t`, `umfpack_dl_*` throughout, pattern-compare on widened arrays) + `sparse_lu_factor.cpp` (`get_lunz/get_numeric` → dl, int64 buffers, guarded narrowing into int32 caches with fallback to the int64 wsolve path). `NativeLU::extract` likewise |
| `HIGHSINT64` | ✅ vendored HFactor stub + full embedded HiGHS build (`BuildHiGHS.cmake`) flipped consistently (shared template sources can't mix ABIs); `scip/lpi/lpi_highs.cpp` patched for the int64 HiGHS API (21 sites, zero behavior change) |
| Eigen `StorageIndex=int64_t` | ⏸ deferred — ABI-wide (240 files + Python + vendored interfaces); int32-overflow guards from Phase 1 stand as the safety net |

### Item 7 — copy chains (closed at approved scope)

- Engine API: by-value `solve()` + moved normalization (3 → 1 full-model copy). ✅ (Phase 1)
- B&C entry: `strict_highs_original_entry_lp` copy made conditional on strict mode. ✅
- Per-node `StandardFormLP` copies → single shared workspace + cuts-augmented dirty flag (restore pristine base only on transition); `active_lp_for_proof` aliases `base_lp` in the no-cuts case. ✅
- Node slimming: `nlp_down/up_scores` are dead in the MILP path but read by `bc_minlp_legacy.cpp` — left in place (full `TreeNode` migration stays deferred; the dead duplicate `bc/core/node_types.hpp` was removed). ⚠️

### Phase 2

| # | Item | Status |
|---|---|---|
| 11 | KKT analyze-once/factorize-many | ✅ scatter-map assembly in `kkt_system.cpp` (`assemble_augmented_kkt_cached`) used by `factor_kkt_sparse` and all three inertia-correction retry sites; pattern fingerprints via `memcmp` on outer/inner index arrays |
| 12 | Augmented `-S·M⁻¹` Newton system | ✅ `IPMOptions::use_augmented_newton` (default off) — 3-block assembly with per-iteration scatter refill, δ_W/δ_C escalation, direct dμ recovery; Gondzio corrector rhs `[0;0;−M⁻¹ε]` and SOC made path-aware; matches condensed path on the test NLP |
| 13 | OpenMP | ✅ `MIPSOLVERS_USE_OPENMP` (libomp on Apple), threshold-guarded parallel SpMV (`ae_mul`/`aet_mul`/`ae_mul_sub`) and theta/residual element-wise loops in the LP IPM. Node-level B&C parallelism (dead `parallel/` stack) stays deferred |

### Phase 3

| # | Item | Status |
|---|---|---|
| 14 | Env-var governance | ⚠️ first step — `BcEnvOptions` (`bc_env_options.hpp/.cpp`) parses the 11 algorithm-affecting toggles once; migrated the 3 centralized helpers + 8 direct sites (strict-HiGHS root pipeline, SCUC dynamic-cut toggles). ~190 debug/trace flags remain env-driven by design (diagnostics, not algorithm choices) — full tree split stays deferred |
| 15 | NETLIB regression suite | ✅ `tests/test_netlib_regression.cpp` (integration tier) + 5 vendored problems (`tests/data/netlib/`, coin-or Data-Netlib, MIT): afiro, adlittle, share2b, stocfor1, kb2 against published optima via the vendored HiGHS MPS reader |

### Bugs found by the NETLIB suite (all fixed)

1. **G-type rows broke the IPM** (`b = +∞` → infinite initial slack → NaN
   barrier). Fixed by one-sided-row normalization (negate to L-type, dual
   sign restored) in both IPM paths — see `numerical_methods.md` §1.
2. **IPM least-squares init produced inf/huge iterates** on near-singular
   initial systems. Fixed: finiteness + magnitude sanity with plain-interior
   fallback.
3. **Dual simplex "Phase I failed" on G-type rows** — `build_standard_form_lp`
   treated `b = +∞` rows as vacuous with `rhs = +∞` (NaN in Phase I). Fixed:
   proper `sense='G'` surplus classification with finite shifted RHS.
4. **Ruiz scaling stalls some degenerate problems** (stocfor1) while being
   required on others (afiro). Fixed: **scaling fallback** — failed scaled
   solves retry with `ruiz_rounds = 0` (`NativeIPMLPAdapter::solve_lp`).

### Validation (final)

- `ctest -L unit`: **10/10** suites (engine_api, lp, milp, ipm, dual_simplex,
  problem_validation, adapter_registry, presolve, l2o_trace,
  numerical_stability — 8 cases/354 assertions).
- Integration: `test_netlib_regression` (220 assertions) passes.
- Pre-existing failures (verified at HEAD via stash-check, unrelated to this
  work): `test_ipopt_parameter_stability` (1 assertion) and
  `test_scuc_module`'s SCED-LP case (embedded HiGHS returns "solve failed" on
  the small binary-fixed SCED LP even at HEAD — separate issue to triage).
- Repo hygiene note: `.gitignore` line 125 has a bare `tests/` entry, so NEW
  files under `tests/` (e.g. `test_netlib_regression.cpp`,
  `tests/data/netlib/`) are ignored by default and need `git add -f`.

## 11. Phase-4 Log — OPF-scale Newton systems (measured)

Driver: the native parity IPM "cannot solve >10000-bus OPF" (PEGASE-13659:
53 s, filter line-search failure; PEGASE-1354 likewise).  Full derivations
in `numerical_methods.md` §9; this is the change log.

| # | Item | Status |
|---|---|---|
| 16 | MUMPS 5.7.3 vendored in-tree | ✅ `mumps/` mirrors the `highs/ scip/ ipopt/ suitesparse/` vendoring convention; `cmake/BuildMUMPS.cmake` builds sequential double-precision `dmumps` from source by default (`MIPSOLVERS_FORCE_BUILD_MUMPS=ON`) — no network, no Homebrew at configure/build/deploy. PORD + mpiseq compiled in; Accelerate BLAS |
| 17 | SuiteSparse extended in-tree | ✅ `suitesparse/` now carries UMFPACK + KLU + BTF next to CHOLMOD; `umfpack_vendored`/`klu_vendored` static targets, vendored include dirs authoritative (Homebrew SuiteSparse block skipped when vendored CHOLMOD is active) |
| 18 | `MumpsSolver` backend | ✅ `engine::MumpsSolver` (RAII `dmumps_c`, sym=2, auto ordering, no internal scaling, cntl(1)=0.01) + `negative_eigenvalues()` (INFOG(12) inertia for Wächter–Biegler). Unit test in `test_numerical_stability.cpp` (`[numerical][mumps]`) |
| 19 | Stale-ABI header bug (root-caused) | ✅ `ipopt/Algorithm/LinearSolvers/` carried hand-vendored **MUMPS-5.6.2-ABI** headers (`dmumps_c.h`, `mpi.h`, `mumps_compat.h`, `mumps_mpi.h`, `smumps_c.h`, commit `41c314a`) that shadowed the real 5.7.3 headers via same-dir include resolution → struct-layout mismatch → Ipopt read `infog[11]=0` ≠ expected inertia → infinite re-factorization loop ("Error in step computation", 0 iterations, 3 failing `test_engine_api` cases). Deleted the stale bundle; embedded Ipopt now compiles against the same 5.7.3 headers as the library. Ipopt then converged case1354 (obj 74069.1, 479 iters, 75.6 s) |
| 20 | Factory policy documented | ✅ `make_default_sparse_solver()` deliberately keeps UMFPACK > KLU > MKL > SuperLU > Eigen: MUMPS absorbs null pivots instead of flagging singularity, which would defeat the generic IPM's δ_C escalation (regression: `test_ipm_solver` δ_C assertion). MUMPS is selected explicitly on well-posed KKTs (parity OPF) |

HybridACDC `optimal_power_flow` side (sibling repo, same investigation):

| # | Item | Status |
|---|---|---|
| H1 | Dense O(n³) warm start | ✅ `dc_warm_start` built a dense 13658×13658 `MatrixXd` (~1.5 GB) + dense LU (~1.7e12 flops) — **94% of total runtime** on case13659. Replaced with sparse triplet assembly + `SparseLU(COLAMD)` on the grounded-Laplacian B′: 48.3 s → 5.5 s |
| H2 | MUMPS as parity-KKT backend | ✅ backend id 4 in `SparseKKTCache`, first in Auto order, `HACDCPF_OPF_LINEAR_SOLVER=mumps` pin; UMFPACK/KLU/Eigen remain escalation failovers. Measured ≈1.75× UMFPACK on the post-H1 remainder (5.5 s vs 9.6 s), matching the §9.3 symmetry prediction (~2×) |
| H3 | Augmented Newton form (the convergence fix) | ✅ `HACDCPF_OPF_KKT_FORM=augmented|condensed`, default augmented (condensed when dense path or niq=0). Condensed `W = Lxx + JhᵀΣJh` reached κ ≳ 1e10 (σ-range ~1.5e10, ‖W‖ ≈ 9e8) → non-descent Newton step → filter stall *with every solver*. Augmented keeps −ZM⁻¹ explicit; Wächter–Biegler δ_W inertia loop via `negative_eigenvalues()` (negevals must equal meq+niq; kick 1e-8·‖Lxx‖, ×8, cap 1e-2·‖Lxx‖). **case1354: fail(25) → converged(110), obj 74069.4 = Ipopt's 74069.1, 2.45 s vs Ipopt 75.6 s (31×)**. Also converges augmented+UMFPACK (no inertia oracle) |
| H4 | Small-case regression sweep | ✅ case14/30/39/57/118/300 all converged with augmented default at their known optima (8081.52 / 576.892 / 41864.2 / 41737.9 / 129661 / 719723), ≤0.12 s each |
| H5 | case13659 augmented | ⚠️ improved, not cured: condensed stalled at iter 59 (obj 1.69e7, 47 s); augmented descends steadily — obj 1.38e7 at the 640-iter cap (447 s), and with δ_W persistence obj **1.17e6 at iter 251** (149 s, 0.59 s/iter) where the filter stalls.  The remaining gap is globalization (SOC/restoration — `numerical_methods.md` §9.5), not linear algebra |
| H6 | δ_W persistence (κ_W⁻ rule) | ✅ inertia loop warm-starts at δ_W⁻/2 instead of 0 (Ipopt practice): case13659 447→149 s, case1354 2.45→2.06 s, small-case sweep unchanged |
| H7 | rte endgame diagnosis | 📋 case1888rte reaches feas 2.1e-5 / grad 4e-6 / comp 1.5e-4 (~4% from the MATPOWER optimum) then the strict `(1−ηα)` filter test fails the last iterations; the best-iterate restoration says "10× tolerance" in its comment but applies 1× — flagged as a documented-but-unimplemented relaxation |

### Validation (Phase 4)

- `ctest -L unit`: 10/10 suites pass after the MUMPS/Ipopt changes
  (`test_engine_api` 92 assertions, `test_numerical_stability` 360
  assertions incl. the MumpsSolver case).
- Pre-existing failures unchanged: `test_ipopt_parameter_stability` (one
  assertion — its `unscaled_dual_feas > 1e-8` bound is calibrated to MUMPS
  5.6.2 numerics; the from-source 5.7.3 stack lands at 2.4e-10, i.e. the
  solver is *more* accurate than the test anticipated — flagged for the
  user, not silently relaxed) and `test_scuc_module`'s SCED-LP case
  (HiGHS "solve failed" at HEAD, unrelated).
- HybridACDC's own ctest suite could not be re-run here: its Release-mode
  configure enforces a clean-tree pin on the sibling MIPSolvers checkout
  (dirty = FATAL by design).  Parity-side validation used a probe binary
  linking the Release `libhacdcpf.a` recompiled with the same flags.

### Phase-5 globalization slice (HybridACDC parity IPM, measured)

Derivations and the per-mechanism evidence: `numerical_methods.md` §9.5.

| # | Item | Status |
|---|---|---|
| P1 | Second-order correction (SOC) in the filter | ✅ on rejection with θ_trial > θ_k, re-solve with `rhs_eq ← −(α·rg + rg_trial)`, composite step once/iter. case13659: 251→377 iters before stall |
| P2 | Iterative-refinement divergence guard | ✅ correction applied only if it provably shrinks the residual — killed the ‖d‖~1e85 blow-ups in the rte endgame |
| P3 | Corrector re-centering | ✅ γ ← max(γ, μ_cur/2), ≤2 retries, triggered at ftb α < 1e-9 (μ-collapse detector) |
| P4 | Catastrophe guard | ✅ no acceptance path may worsen grad/comp >100× (was: comp 1.1e-3→7.6e8 accepted → μ blew to 1e9) |
| P5 | Acceptable-level termination | ✅ best-iterate acceptance now implements the documented relaxation (Ipopt convention 100× tol, honest status string) — previously the comment said "10×" but the code applied 1× |
| P6 | Outcomes | case1951rte **converged at optimum** (81737.8); case1888rte converged-acceptable at 27224.8 (bad basin; pre-guard trajectory once saw 59790.2 — knife-edge basin sensitivity, documented in §9.5(c)); case1354 cured (74069.4, 31× Ipopt time); small sweep unchanged; case13659 best obj 1.17e6, still not converged — remaining gap is architectural (restoration phase / (θ,φ) barrier-merit filter), see §9.5 |

### Validation (Phase 5)

- All changes are in the sibling HybridACDC `optimal_power_flow/parity_ipm.cpp`
  (probe-linked against Release `libhacdcpf.a` with identical flags; the
  Release configure there still requires a clean MIPSolvers tree to re-run
  their ctest suite).
- Sweep after the slice: case14/30/39/57/118/300 converged at known optima;
  case1354 converged (113 iters, 2.2 s); rte cases as in P6.
- MIPSolvers side untouched by this slice; `ctest -L unit` remains 10/10.

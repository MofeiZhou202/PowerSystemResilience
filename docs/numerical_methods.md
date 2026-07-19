# Numerical Methods in MIPSolvers — Derivations and Design Guidance

This document collects the mathematical derivations behind the numerical
work of Phases 0–3 (see `docs/code_quality_evaluation.md` for the itemized
logs).  It is written for engineers who need to understand *why* the code
looks the way it does before changing it — each section states the problem,
derives the method, and gives the engineering guidance that follows.

Contents
- [1. One-sided (G-type) constraint rows](#1-one-sided-g-type-constraint-rows)
- [2. Ruiz equilibration](#2-ruiz-equilibration)
- [3. Iterative refinement](#3-iterative-refinement)
- [4. Analyze once, factorize many](#4-analyze-once-factorize-many)
- [5. Condensed vs augmented Newton system](#5-condensed-vs-augmented-newton-system)
- [6. Index width: where 2³¹ actually breaks](#6-index-width-where-2%C2%B3%C2%B9-actually-breaks)
- [7. Sparse direct backends: simplicial, supernodal, multifrontal](#7-sparse-direct-backends-simplicial-supernodal-multifrontal)
- [8. Memory-bandwidth model of the copy chains](#8-memory-bandwidth-model-of-the-copy-chains)

---

## 1. One-sided (G-type) constraint rows

**Problem.**  `LPModel` admits two-sided rows `lhs ≤ aᵀx ≤ b`.  A
*one-sided* row with `b = +∞` (a `G`-row, `aᵀx ≥ lhs`) breaks the IPM-LP
kernel: it forms the row equation `aᵀx + s = b` with slack `0 ≤ s ≤ b − lhs`,
so `b = +∞` poisons the slack (`s = b − aᵀx = +∞`), the barrier gap
`g_u = ub − x_s = kBig − ∞ < 0`, and finally the normal-equation diagonal
`θ = z_l/g_l + z_u/g_u`, which goes negative — the Cholesky "succeeds" on a
non-PD matrix and the iterate goes NaN.

**Derivation.**  The row set is invariant under negation of any single row.
For a `G`-row, replace `aᵀx ≥ lhs` by the equivalent `L`-row

```
−aᵀx ≤ −lhs  ⟺  (−a)ᵀx + s = −lhs,  s = (−lhs) − (−a)ᵀx = aᵀx − lhs ≥ 0,
```

so the slack is exactly the surplus, bounded below by 0 and unbounded above
(`ub_s = kBig`).  Dual sign: with the row flipped, stationarity of the
Lagrangian in the flipped form gives `c = (−A)ᵀŷ + …`, i.e. the flipped
dual `ŷ_i` equals `−y_i` of the original row.  Hence `constraint_duals` are
negated back at extraction.

**Guidance.**  Normalization (flip `G`-rows to `L`-type, dual sign restored
at the end) belongs at the solver's input edge — it is implemented in both
`solve_lp` and `prepare_for_node_solves` (cached B&C path) and, for the
simplex, natively inside `build_standard_form_lp`.  Never special-case
one-sided rows inside the iteration; normalize once, keep the iteration
uniform.

---

## 2. Ruiz equilibration

**Problem.**  Rows/columns of `A` spanning `1e−10…1e10` make the normal
matrix `N = A·Θ·Aᵀ` effectively singular in double precision.

**Derivation.**  Choose diagonal `D_r ≻ 0`, `D_c ≻ 0` and the similarity
transform

```
min (D_c c)ᵀx̂   s.t.  (D_r A D_c) x̂ + ŝ = D_r b,  ŝ ≥ 0,
                       lb/D_c ≤ x̂ ≤ ub/D_c,
```

which is the *same* LP under `x = D_c x̂`, `y = D_r ŷ`, `z = ẑ/D_c` (verify
stationarity: `(D_c c) − (D_c Aᵀ D_r)ŷ = 0 ⟺ c − Aᵀ(D_r ŷ) = 0`).
Ruiz iteration alternates row and column ∞-norm equilibration with
`1/√‖·‖∞` factors per round — a Sinkhorn-type alternating projection that
drives every row/column ∞-norm toward 1 geometrically (factor
`√(r_new) = √(‖r‖∞/√‖r‖∞) = ‖r‖∞^{1/2}` per round, so norms converge to 1
at rate 1/2 per round in log scale).  A correct implementation must
accumulate the cumulative `D_r, D_c` **once per row/column** — applying a
row factor per *nonzero* squares `D_r` each round and silently changes the
problem (the Phase-1 bug the numerical tests caught).

**Failure mode.**  Equilibration is a heuristic preconditioner, not a free
lunch: on degenerate problems (e.g. NETLIB `stocfor1`) aggressive rounds
produce a *worse*-conditioned scaled system — the iterate wanders far off
the central path (barrier `μ` explodes to `1e13+`).  Meanwhile
`afiro` *requires* scaling (unscaled IPM lands 1% off-optimal).

**Guidance.**  Keep the solver's scaling **on by default** with a
**scaling fallback**: if a scaled solve fails to converge, retry with
rounds = 0 (implemented in `NativeIPMLPAdapter::solve_lp`).  Never change
default scaling without a NETLIB-subset check on both regimes.

---

## 3. Iterative refinement

**Problem.**  A factorization `Â ≈ A` with residual `r = b − Âx` has
forward error `‖x − x*‖ ≲ κ(A)·‖r‖` — at `κ ~ 1e12`, double precision
(`u ≈ 2.2e−16`) loses ~4 digits per solve; FT-update chains and
ill-conditioned KKT systems accumulate this.

**Derivation (Wilkinson).**  With `x₀` computed and `r₀ = b − A x₀`,
solve `A δ = r₀` with the *same* (cheap, inaccurate) factorization and set
`x₁ = x₀ + δ`.  Then `r₁ = b − A x₁ = (A − Â)δ + O(u²)`, and

```
‖x₁ − x*‖ / ‖x*‖  ≲  q · ‖x₀ − x*‖ / ‖x*‖ ,   q ≈ κ(A)·u < 1 ,
```

so each refinement step multiplies the error by `q < 1` — convergence to
working precision whenever `κ(A)·u < 1`.  The cost is one SpMV (`r`) plus
one back-solve (`δ`) per step: negligible against factorization.

**Guidance.**  Refinement should be **residual-gated**, not always-on:
evaluate `‖r‖∞ ≤ τ·max(1,‖b‖∞)` first (τ ≈ 1e−12) and only pay the
back-solve when the residual demands it.  This is what the KKT path
(≤2 steps), the LP-IPM normal equations, and UMFPACK's `IRSTEP=2` all do
now.  When `q ≥ 1` (matrix too ill-conditioned), refinement does not
converge — escalate regularization instead (`δ_W`/`dyn_reg`), don't keep
refining.

---

## 4. Analyze once, factorize many

**Problem.**  IPM assembles its Newton matrix every iteration; naive
triplet assembly (`setFromTriplets`) costs `O(nnz·log nnz)` plus
allocation churn, and re-does symbolic factorization.

**Derivation.**  Every Newton matrix in these solvers has the form
`K(v) = P·diag(v)·Pᵀ + R` with **fixed** sparsity pattern: `P` (Jacobians,
slack identities) is constant; only the *values* (barrier `Θ`, `δ_W`,
Hessian) change.  Hence the pattern can be analyzed once (fill-reducing
ordering + symbolic factorization), and per-iteration assembly reduces to
a *scatter map*: for each source entry `k`, a precomputed destination
index `pos[k]` in the factor's CSC value array, so

```
values[pos[k]] += v[k]      — O(nnz) total, zero allocation.
```

Building the map is one `O(nnz · log)` pass over the pattern (binary search
per entry); amortized over `O(10–10³)` iterations it is free.

**Guidance.**  Any new IPM-style kernel should follow the
analyze-once/factorize-many contract: pattern assembly once, values via a
scatter map, symbolic factor reused, numeric factor per iteration.  This
is implemented for LP-IPM (banded/sparse paths), KKT
(`assemble_augmented_kkt_cached`), and the augmented Newton assembler.
Watch for value-index confusion: `InnerIterator::index()` is a *row*
index, not a storage offset — use direct CSC loops (`outerIndexPtr`).

---

## 5. Condensed vs augmented Newton system

**Problem.**  The primal-dual NLP IPM condenses inequality multipliers
into the Hessian: `W = H + J_hᵀ·diag(μ/s)·J_h`, then solves
`[W, J_gᵀ; J_g, 0]`.  The product `J_hᵀDJ_h` (i) costs a sparse triple
product per iteration, (ii) fills in (fill of `J_hᵀJ_h` can be quadratic
in row counts), and (iii) squares the Jacobian's condition number.

**Derivation.**  Newton equations (`M = diag(μ)`, `S = diag(s)`):

```
H dx + J_gᵀ dλ + J_hᵀ dμ = −r_d
J_g dx                   = −r_eq
J_h dx + ds              = −r_ineq
M ds + S dμ              = μ̄e − Sμ
```

Eliminating `ds, dμ` gives the condensed form with `W = H + J_hᵀ(M/S)J_h`.
Keeping `dμ` unknown and dividing the last row by `M` gives instead the
augmented symmetric indefinite system

```
[ H + δ_W I   J_gᵀ     J_hᵀ ] [dx ]   [ −r_d                      ]
[ J_g        −δ_C I     0   ] [dλ ] = [ −r_eq                     ]
[ J_h          0      −SM⁻¹ ] [dμ ]   [ −r_ineq − M⁻¹(μ̄e − Sμ)    ]
```

with a *negative diagonal* `−S·M⁻¹ = −diag(s_i/μ_i)` in the (3,3) block —
no product is ever formed.  Conditioning: eliminating the (3,3) block
reproduces the condensed matrix, so by block congruence

```
inertia(augmented) = inertia(condensed) + (0, m_ineq, 0),
```

i.e. the usual descent condition `inertia = (n, m_eq, 0)` on the condensed
form is exactly `inertia = (n, m_eq + m_ineq, 0)` on the augmented form —
the same Wächter–Biegler `δ_W` correction applies unchanged.  The Gondzio
corrector rhs in this form is `[0; 0; −M⁻¹ε]` (derive by substituting the
correction equations), and SOC re-uses the same rhs with the equality
block replaced.

**Guidance.**  The augmented form (`IPMOptions::use_augmented_newton`) is
the right choice when `J_h` is wide/dense or `nnz(J_hᵀJ_h) ≫ nnz(J_h)`;
the condensed form wins when the extra `m_ineq` factorization dimension
dominates (narrow banded `J_h`).  Both are assembled analyze-once (see §4).

---

## 6. Index width: where 2³¹ actually breaks

**Problem.**  The "int32 ceiling" is not the dimension `n` — `n = 10⁶`
fits easily.  It is the **factor nonzero count**: `nnz(L)+nnz(U)` of a
million-row basis can exceed `2³¹`, at which point `int` counters wrap
*silently* (negative sizes → garbage allocations).

**Guidance.**  The 64-bit ceiling must be raised at the *factorization
interface* first: UMFPACK via `umfpack_dl_*` (`int64_t` indices), HiGHS
`HIGHSINT64`, CHOLMOD `cholmod_l_*`.  Internal CCS/CSR caches may stay
32-bit as long as *values* are indices `< n ≤ 2³¹`, but every *count* and
every interface array must be 64-bit, and every narrowing conversion must
be guarded (loud failure, not truncation).  A full `Eigen
StorageIndex=int64_t` migration is a separate ABI-wide effort; the
guards + `dl` interfaces close the silent-overflow hole now.

---

## 7. Sparse direct backends: simplicial, supernodal, multifrontal

**Problem.**  Choosing the open-source sparse direct solver for the SPD
(normal-equation) and indefinite (KKT) paths.

**Guidance.**
- **Simplicial LDLᵀ** (Eigen, CHOLMOD simplicial): column-at-a-time, BLAS-1
  — memory-latency bound, fine for small/sparse, hopeless at scale.
- **Supernodal** (CHOLMOD supernodal, this project's default SPD backend):
  groups columns into dense panels and factors them with BLAS-3
  (`dgemm`/`dsyrk`/`dtrsm`) — the right default: analyze-once symbolic,
  numeric-per-iteration, portable (SuiteSparse, no vendor lock).
- **Multifrontal / Bunch–Kaufman** (MUMPS, MA57, Pardiso): required for
  *indefinite* KKT systems at scale (the augmented Newton form of §5).
  MUMPS is the open-source counterpart — vendored in-tree (offline build,
  no Homebrew) and wired both into the embedded Ipopt and as the
  `MumpsSolver` backend of the parity-IPM OPF path; the full derivation of
  why this class wins on OPF KKTs is §9.
- **Apple Accelerate** sparse Cholesky: on Apple Silicon it beats CHOLMOD
  (~2.7× on the measured normal equations) — keep it first on macOS;
  CHOLMOD is the portable primary elsewhere (measured priority
  `Accelerate > CHOLMOD > Eigen`).

---

## 8. Memory-bandwidth model of the copy chains

**Problem.**  Each LP solve copied the full model 3–5 times (API layer,
B&C entry, per-node standard form); B&C nodes each stored 4 full-length
`Eigen::VectorXd` + ~15 vectors — at `n = 10⁶` with 10⁴ open nodes this is
~160 GB, and even at moderate `n` the copies are pure memory-bandwidth
cost dominating small solves.

**Guidance.**  (i) Pass problems by const reference / move; normalize in
place once.  (ii) Node workspaces are *shared and mutated incrementally*
(`update_standard_form_bounds` applies only changed bounds), restoring
from the immutable tree base only when a cuts-augmented state is left —
never copy per node by default.  (iii) Long-lived caches (scatter maps,
symbolic factorizations, CHOLMOD analyses) are built once per problem and
reused per iteration (see §4).  These are the changes made in Phase 1/2;
the remaining deep item is migrating the B&C node representation itself
to the incremental `TreeNode` (bound-change deltas instead of full
`lb/ub` arrays) — a rewrite of the legacy tree loop, deferred deliberately.

---

## 9. OPF-scale Newton systems: measured bottleneck hierarchy

**Problem.**  The native parity IPM "could not solve" the PEGASE-13659 OPF
(>10⁴ buses): 53 s and a filter line-search failure.  This section records
the actual bottleneck hierarchy found by profiling and the derivation of
each fix, in the order in which they bind.  The lesson is that three
*distinct* asymptotic mechanisms were stacked; fixing the linear solver
alone moved total runtime by <10%.

### 9.1 First bind: an O(n³) dense warm start (engineering asymptotics)

`build_initial_point`'s DC warm start solved `B'θ = P_inj` — the reduced
network Laplacian system — by materializing a **dense** `(nb−1)×(nb−1)`
matrix and calling `partialPivLu`:

```
memory  = (nb−1)² · 8 B          ≈ 1.5 GB   at nb = 13659
work    = (2/3)(nb−1)³ flops     ≈ 1.7e12   at nb = 13659
```

Profiled share of the whole OPF solve: **94%** (48.3 s of 51 s), all inside
Eigen's dense GEMM/LU kernels.  But `B'` is the imaginary part of `Ybus` —
a grounded graph Laplacian with `O(nnz)` nonzeros and, for near-planar
power grids, `O(√n)` separators, so sparse LU with a fill-reducing ordering
costs `~O(n^{3/2})` (§9.3).  Replacing the dense LU with triplet assembly +
`SparseLU(COLAMD)` took case13659 from 48.3 s to 5.5 s.  *Any* remaining
claim about "the KKT factorization dominating" had to be re-measured after
this fix — the earlier >99%-factorization figure came from a regime where
this warm start had already been paid once per solve and amortized away.

### 9.2 Second bind: the condensed Newton form (the solvability barrier)

With the warm start sparse, all linear solvers (MUMPS/UMFPACK/KLU) produced
**bit-identical trajectories** and the same failure: the filter line search
stalls — 15 backtracks to α ≈ 1e-6 with *no* descent in any filter axis
(feasibility, stationarity, complementarity).  Measured at the stall:

```
σ = μ/z  spans  [2e-5, 3e5]      (dynamic range ~1.5e10)
‖W‖∞     ≈ 9e8 ,  W = Lxx + JhᵀΣJh (condensed Hessian, §5)
```

The condensed form (§5) squares the Jacobian's condition number and
injects `Σ = diag(μ/z)` *into* `W`: by the stall, `κ(W) ≳ 1e10`, so the
computed Newton direction carries only ~5–6 correct digits in the worst
subspace — it is numerically perpendicular to the true direction
("accurately-solved but wrong step": small KKT residual, worthless step).
No linear solver can fix this, because the *system* is wrong, not its
solution — exactly the §5 conditioning argument, here measured.

The augmented form keeps `−ZM⁻¹` explicit:

```
[ Lxx+δW·I   Jgᵀ      Jhᵀ  ] [dx]   [−rd ]
[ Jg        −δC·I     0   ] [dλ] = [−req]
[ Jh          0     −ZM⁻¹ ] [dμ]   [rhs3]
```

Conditioning drops from `O(κ(Jh)²·range(σ))` to `O(κ(Jh)·range(σ))` and no
triple product is formed (also removing its per-iteration fill and flops).
Result on case1354pegase: **fail (iter 25, obj 435 567) → converged
(iter 110, obj 74 069.4)** — matching embedded Ipopt's objective
(74 069.1) to 4e-6, 31× faster than Ipopt (2.45 s vs 75.6 s).  The
augmented form alone already converges with LU backends (UMFPACK: iter
104, same objective); the LDLᵀ backend adds the inertia oracle:

**Wächter–Biegler δ_W loop.**  With the reduced Hessian positive definite,
`inertia(K_aug) = (n, meq+niq, 0)`.  An LDLᵀ factorization reports the
negative-pivot count for free (§9.4); when `negevals ≠ meq+niq` the reduced
Hessian is indefinite and the step need not be descent, so δ_W is
escalated (kick `1e-8·‖Lxx‖`, ×8 per retry, cap `1e-2·‖Lxx‖`) and the KKT
is refactorized.  This replaces the previous blind regularization ladder
which accepted the first factorization that did not fail — including ones
with wrong inertia, which is precisely how non-descent steps were
accepted.

### 9.3 Third bind: factorization backend class (the remaining constant)

After 9.1–9.2, the per-iteration cost is a healthy mix of assembly,
factorization, and solves.  Here the §7-class theory applies and is now
measurable: on the same filled graph, symmetric LDLᵀ costs ~½ the flops
and fill of unsymmetric LU (one triangle eliminated, Bunch–Kaufman 1×1/2×2
pivoting with the `(1+√17)/8` threshold is backward stable on indefinite
systems), and nested dissection on near-planar grid graphs gives fill
`O(n log n)`, work `O(n^{3/2})` (George 1973; Lipton–Tarjan 1979) versus
minimum-degree's degradation toward `O(n²)` on large meshes.  Measured on
case13659 (condensed form, same failure, so trajectories are comparable):
MUMPS 45.8 s vs UMFPACK 50.1 s pre-warm-start-fix; **5.5 s vs 9.6 s**
after — i.e. LDLᵀ ≈ **1.75×** on the factorization-bound remainder,
consistent with the ~2× symmetry prediction.

**Inertia vs singularity flags — why one solver cannot serve both paths.**
The parity-IPM OPF KKT is well-posed after Ruiz equilibration + δ_W, so
MUMPS is its default backend.  The *generic* IPM instead drives δ_C
escalation off singularity *flags*; MUMPS absorbs null/tiny pivots
(`CNTL(3)`, `ICNTL(24)`) instead of failing, which would defeat that
machinery — so `make_default_sparse_solver()` keeps UMFPACK > KLU > … and
MUMPS is selected explicitly where the KKT is known well-posed.  This
asymmetry is a consequence of the algorithms, not of preference.

### 9.4 Inertia is free with LDLᵀ

`K = LDLᵀ` is a congruence, so by Sylvester's law `inertia(K) = inertia(D)`
and the negative eigenvalue count is the negative-pivot count of `D`
(MUMPS `INFOG(12)`; exposed as `MumpsSolver::negative_eigenvalues()`).
LU backends cannot provide inertia at all — with them the IPM must
regularize blindly.  Free per-iteration inertia is a first-order
algorithmic advantage of symmetric-indefinite factorization for interior
point methods, not an implementation detail.

### 9.5 Globalization in the mid/endgame: mechanisms added and what they did

With 9.1–9.3 fixed, the large cases fail *differently* — the failure moved
from the linear algebra to the globalization.  Trajectory instrumentation
(MUMPS pivot counts, direction norms, per-trial filter axes) isolated three
distinct endgame mechanisms; for each, the standard theory remedy was
implemented and measured.  Outcomes are honest: some cure, some only move
the stall.

**(a) Maratos rejection — second-order correction (SOC).**  When the first
trial is rejected with θ_trial > θ_k, re-solve the same factorization with
`rhs_eq ← −(α·rg + rg(x+αdx))` and try the composite step once per
iteration (Wächter–Biegler §3.2; mirrors the engine IPM's
`use_second_order_correction`).  case13659: 251 → 377 iterations before the
stall, same final objective — more progress per unit regularization, no
cure.

**(b) Direction blow-up — re-centering + refinement guard.**  Near the
barrier boundary κ(KKT) ~ 1/μ; at the stall the solved direction reached
‖d‖ ~ 1e85, so even α = 1e-10 trials exploded (residuals ~1e75).  Two
defects compounded: iterative refinement with no divergence guard
(amplifies near-null-space noise — now applies a correction only when it
provably shrinks the residual), and μ-collapse past the useful limit —
the corrector now re-centers (γ ← max(γ, μ_cur/2), ≤2 retries) when the
fraction-to-boundary limit drops below 1e-9.  The 1e85 blow-ups are gone;
trajectories changed but the stall itself is not removed.

**(c) Poisoned acceptance — catastrophe guard + acceptable termination.**
The filter's feasibility axis accepted steps regardless of dual-axis
damage (comp 1.1e-3 → 7.6e8 accepted at rte iter 104, blowing μ to 1e9
and killing the run).  Every acceptance path now requires
grad/comp ≤ 100× current.  And the best-iterate restoration — whose
comment promised "10× tolerance" but applied 1× — now implements
Ipopt's acceptable-level convention: 100× primal/dual tolerance with
complementarity inside comp_tol, reported honestly as
`converged (acceptable tolerance, best iterate)`.

**Measured outcomes (final build).**  case1951rte converges at obj
81737.8 (the MATPOWER optimum).  case1888rte converges *acceptably* at
obj 27224.8 — a valid KKT-acceptable point but a worse basin than the
59790.2 its pre-guard trajectory once touched: on knife-edge nonconvex
cases, step-level changes shift basins chaotically; the guards are kept
because accepting 6-orders-of-magnitude axis damage or refinement-diverged
noise is indefensible even when luck once made it useful.  case1354pegase
remains cured.  Small-case sweep unchanged.

**What remains open.**  case13659 progresses steadily (best obj 1.17e6)
but does not converge: its midgame crawl (α ~ 1–2% fraction-to-boundary)
outruns the iteration budget.  The mechanisms that would close this are
architectural, not local: a restoration phase (Ipopt-style infeasibility
minimization with bound-relaxing slacks) for when the filter fails, and
a (θ, φ) merit filter whose φ is the *barrier objective* — the Newton
direction is descent on φ by construction with correct inertia, whereas
descent on the raw stationarity axis used here is not guaranteed at
κ ~ 1/μ.  That is the principled explanation for why Ipopt's filter
terminates these cases and this one does not, and it is the next slice.
Per-iteration cost is now ~55% MUMPS factor+solve, ~25% triplet
re-assembly + per-iteration Ruiz rescaling (the engine's scatter-map
assembly, §4, is the documented removal), ~20% model evaluations.

# Numerical Methods in MIPSolvers — Derivations and Design Guidance

This document collects the mathematical derivations behind the shared numerical
kernel (see `docs/solvers.md` for the current solver inventory and audit path).
It is written for engineers who need to understand *why* the code
looks the way it does before changing it — each section states the problem,
derives the method, and gives the engineering guidance that follows.

The maintained dual-simplex contract is now consolidated in `docs/solvers.md`,
including the sign convention, Phase I/II state machine, BFRT transactions,
certificate requirements, implementation boundaries, and test gates.

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

For KKT systems, a correction is committed only when it strictly decreases
the original assembled-system residual. This preserves the best available
Newton direction when the factorization is near the limit of double precision
and prevents a failed refinement attempt from making the iterate worse. If no
residual-decreasing correction exists, the caller should treat the solve as a
regularization problem, not as a request for more refinement passes.

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

The implementation factors a symmetrically equilibrated but algebraically
identical form. With

```
T = diag(I, I, sqrt(M/S)),
Tᵀ K_aug T = [ H+δ_W I, J_gᵀ, J_hᵀ sqrt(M/S);
                J_g,      −δ_C I, 0;
                sqrt(M/S)J_h, 0, −I ].
```

The transformed right-hand-side tail is `sqrt(M/S) r_h`, and the physical
multiplier direction is recovered as `dμ = sqrt(M/S) dμ_scaled`. This diagonal
congruence preserves Sylvester inertia and the exact Newton direction, while
removing `s_i/μ_i` from the factor pivots. MUMPS `SYM=2` also uses its symmetric
automatic scaling (`ICNTL(8)=77`), which is another inertia-preserving diagonal
equilibration. In `NewtonFormulation::Auto`, exhaustion of condensed inertia
correction may switch to this augmented system; an explicitly forced condensed
form still fails loudly instead of changing formulation.

**2026-09-04 comparison with the prediction.** The first H13 result contradicted the prediction
that augmented factorization would close the condensed inertia failure: MUMPS
reported 97 numerical null pivots, and `δ_W` escalation could not affect them.
The `sqrt(M/S)` congruence reduced the count to 46, identifying the remaining
deficiency as equality-side row scaling rather than curvature or incorrect
elimination. Enabling MUMPS symmetric scaling removed the false deficiency; H13
then converged in 14 rather than failing after 60 condensed factorizations.
The four-decade inequality-row regression in `test_numerical_stability` checks
the resulting augmented solution against the unscaled reference. The eight-
decade variant is not claimed: it exposed globalization sensitivity outside the
current OPF scaling regime even though the factor congruence remains exact.
Release validation through the HySim consumer command
`./build/macos-release/phase_hybrid_opf_benchmark all parambench native quiet`
measured primal-dual-slack/primal-only median speedups of 5.661x, 2.848x, 1.383x,
and 1.488x on H13/H34/H123/H8500, with numeric factorizations changing from
47/27/30/100 to 4/7/20/131. The geometric mean of the four speedups was 2.400x,
above the fixed required value of 1.20x. This used Release, AppleClang 21,
arm64 macOS 26.5.2, dirty HySim
`ef5f1f6b` and MIPSolvers `a5d614b`; the source CSV is
`output/benchmarks/paper_native_ipm_parametric_warmbench.csv` in HySim.

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

---

## 10. OPF-specific IPM design: problem characteristics and literature-grounded choices

**Why a dedicated analysis.**  Large AC-OPF problems are not generic NLPs.
Their interior-point behavior is dictated by a handful of structural
characteristics; the state of the art (Ipopt, KNITRO, MIPS/MATPOWER,
BELTISTOS, MadNLP/HyKKT on GPUs) is precisely the set of methods that
exploit them.  This section records the characteristics, what the
literature does with them, and which of those mechanisms this codebase now
implements.

### 10.1 The six characteristics

- **C1 — Network-induced sparsity and separators.**  The power-balance
  Jacobian has O(bus degree) ≈ 3–6 nonzeros per row on a near-planar
  graph ⇒ O(√n) separators ⇒ nested-dissection ordering gives fill
  O(n log n), work O(n^{3/2}) (§9.3).  This is what makes direct KKT
  factorization viable at all at 10⁴–10⁵ buses.
- **C2 — (Block-)diagonal Hessian.**  Generation costs are separable
  quadratics; the only curvature coupling comes from branch-flow limits.
  The (1,1) KKT block is diagonal-dominant — cheap to regularize (δ_W·I)
  and benign for frontal solvers.
- **C3 — Box-dominant inequalities.**  Most inequalities are variable
  bounds (Pg, Qg, Vm) ⇒ Jh is mostly a signed selection; branch limits
  add sparse quadratic rows.  The augmented (3,3) block is exactly
  diagonal; the condensed JhᵀΣJh is diagonal-dominant + sparse
  corrections.
- **C4 — Sparse active set, occasional degeneracy.**  Active constraints
  at solution are generator limits and a few branch flows; LICQ usually
  holds, but parallel lines / identical generators create near-dependent
  Jacobian rows — the matrices where MUMPS' null-pivot absorption vs
  UMFPACK's singularity flagging diverge (§9.3's "one solver cannot serve
  both paths").
- **C5 — Structured barrier ill-conditioning.**  κ(KKT) = Θ(1/μ), but the
  ill-conditioning is *structured*: the large eigenvalues live on
  range(Aᵀ) (the active-Jacobian range) and the induced solution error
  concentrates harmlessly there, provided the centrality conditions
  s_i·v_i stay balanced (Wright 1998; Pacaud–Shin–Montoison–Schanen–
  Anitescu 2024, arXiv:2405.14236, Thm 4.2 — "structured ill-conditioning
  counterbalances the accuracy loss").  Their precision ceiling on
  pivoting-free condensed GPU solves is ε^{1/4} ≈ 1e-4 — independent
  evidence that the ~1e-4 "acceptable level" is a real, measurable
  regime, not a cop-out.  Two consequences implemented here: iterative
  refinement needs a divergence guard (their recommended Richardson on
  the unreduced system; our guard keeps the correction only when the
  residual provably shrinks), and Gondzio correctors (which rebalance
  s_i·v_i) are load-bearing, not optional.
- **C6 — Nonconvexity and basin sensitivity.**  AC balance equations are
  quadratic equalities ⇒ nonconvex feasible set, multiple local optima
  (rte cases notoriously).  Step-level perturbations shift basins
  chaotically; convergence machinery must therefore be judged by
  invariants (no poisoned axes, no diverged refinement), not by which
  basin a lucky trajectory once found.

### 10.2 What the literature does, mapped to this code

| Mechanism | Source | Status here |
|---|---|---|
| MA57/PARDISO-class LDLᵀ + inertia control is the reliability winner on OPF KKTs | Kardos–Kourounis–Schenk–Zimmerman 2020 (arXiv:1807.03964) | ✅ MUMPS backend + Wächter–Biegler δ_W via `negative_eigenvalues()` |
| 1e-6 tolerance practice; "acceptable" termination standard; <1e-9 numerically unstable | ibid., §5.2 | ✅ tol=1e-6 + acceptable-100× with honest status |
| (θ,φ) filter + switching condition + Armijo + SOC + restoration = the globalization that terminates hard cases | Wächter–Biegler 2006 (Ipopt) | ✅ (θ,φ) filter with domination history, switching/Armijo, SOC, bounded restoration (§9.5–10.3) |
| Filter must have *history*: current-point-only acceptance lets the iterate wander (600+ iterations of φ-wiggles with slow θ/comp degradation observed on case1888rte) | Wächter–Biegler §3.2 (domination of all previous pairs) | ✅ history with φ re-evaluated at current γ |
| Structured ill-conditioning ⇒ condensed form viable for OPF on pivoting-free architectures; refinement on the unreduced system | Pacaud et al. 2024 (arXiv:2405.14236); Wright 1998 | ✅ refinement divergence guard; augmented form retained (CPU LBLᵀ is available, so no need to accept the ε^{1/4} ceiling) |
| Dynamic barrier updates (relax central-path following; global-local unified convergence) | Armand–Benoist–Orban 2008/2013; Friedlander–Orban 2012 | ❌ tried, negative: the θ_S drop-in (γ = μ(1−α_aff) + α_aff·μ²/μ₀) broke convergence on case118/1354/1888rte (3/3).  Cause: ABO is a Newton step on the *augmented (w,μ) system* with its own μ-line-search; transplanted onto a Mehrotra skeleton it loses the σ³ self-limiting property (γ ≤ μ_cur) and, fed by the re-centering boost, μ can pump upward and wander.  Kept env-gated (`HACDCPF_OPF_MU_STRATEGY=abo`, default mehrotra); a faithful port needs the full augmented-system treatment — deferred |
| Initial guess matters; MATPOWER default start is competitive; DC-OPF warm start is the cheap closer point | Kardos et al. §5.3 | ⚠️ tried, case-dependent: DC-OPF start + 5% interior shift (LP vertices are interior-hostile IPM starts) — case1888rte 168→130 iters, case1354 88→302 iters (**worse**), case118 neutral, case13659 obj 2.7e6→1.78e6 but viol 3.8→6.6, still capped (528 s incl. 146 s DC solve).  The existing physics-informed initializer already supplies the valuable part (DC-consistent θ + *interior* proportional dispatch); the LP-vertex dispatch adds boundary-exactness, which IPMs dislike.  Kept probe-gated, not a library default |
| Structure-exploiting OPF IPM variants (BELTISTOS) use Ipopt algorithms + pivoting/scaling control | Kardos et al. | ✅ via MUMPS ICNTL/CNTL choices |

### 10.3 The remaining engineered pieces and their exact theory

- **(θ,φ) filter**: accept iff θ decreases by (1−γ_θ) or φ decreases by
  γ_φ·θ against the current pair AND every history pair; near feasibility
  (θ ≤ 1e-4) with genuine φ-descent, require Armijo on φ.  φ is the
  barrier merit with the current corrector γ; with certified KKT inertia
  the Newton direction is descent on φ — which the raw stationarity axis
  cannot promise at κ ~ 1/μ (this is the precise reason the old 3-axis
  filter stalled with a *good* direction).
- **Bounded restoration**: on total filter failure, one Gauss-Newton-like
  feasibility step with (1,1) := ρI (ρ = 1e-8·‖Lxx‖), θ-Armijo +
  dual-axis bounds; repeated failures give repeated restoration steps.
  Full Ipopt restoration reformulates the NLP with elastic variables —
  this bounded variant reuses the same KKT machinery at one extra
  factorization per call.
- **Catastrophe guard**: no acceptance path may worsen grad/comp >100× —
  the filter's φ-axis otherwise certifies steps that poison μ.

### 10.4 Measured state after the literature slice

- case1888rte: converged at obj 59771.2 (≈ the MATPOWER reference
  59790.8, −0.03%), 152 iters — previously: fail, or 640-iteration
  wander to 56714.5.
- case1951rte: converged at obj 81757.8, 47 iters, 1.9 s.
- case1354pegase: converged at obj 74069.4 (= Ipopt's 74069.1), 88
  iters, 2.2 s (Ipopt: 75.6 s).
- Small sweep (14/30/39/57/118/300): all converged at known optima,
  violations ≤ 1.5e-5.
- case13659pegase: still open.  With the (θ,φ) filter + history +
  θ_max + the vacuous-step rule (α < 1e-7 declared unusable — at such
  steps the filter margins ~1−1e-12 are numerically vacuous and the IPM
  random-walks toward an infeasible barrier-stationary point), the raw
  violation dropped 30 → 3.77 but the run still caps at 640 iterations
  (327 s, obj 2.7e6).  The midgame convergence *rate* — ftb-limited
  α ~ 1–2% over thousands of KKT unknowns — is what the dynamic-μ
  (Armand–Benoist–Orban) and warm-start items of §10.2 must supply;
  globalization alone cannot.  Also still open: case2869/9241pegase
  (same family, same mechanism); all rte cases and case1354pegase
  converge at their reference optima.

---

## 11. The AC-feasible start: why it unlocks stressed grids (measured)

**Observation (Phase 8).**  The plain AC power flow converges from the
case's own operating point even on the stressed PEGASE grids (case13659:
9 NR iterations, residual 5e-12; case9241: 16 iters) — those grids ARE
AC-feasible.  Yet the parity IPM could not converge on them from the
physics-informed start (DC-θ + interior dispatch): the trajectory leaves
the feasible basin midgame (objective undershoot, filter death).
Starting instead from the solved power flow (θ ≈ 1e-12 at iteration 0),
**case13659pegase converges at the reference optimum obj = 386107 in 189
iterations / ~40 s** (`ACOPFOptions::ac_pf_warm_start`).

**Derivation.**  The (θ,φ) filter's two regimes behave differently by
region: in the infeasible region the θ-axis dominates and acceptance is
fragile (Maratos, vacuous tiny-α, restoration loops — §9.5, §10.3); near
feasibility (θ ≤ θ_min) the switching condition certifies steps by Armijo
on the barrier merit φ, and with certified KKT inertia the Newton
direction is *provably* descent on φ.  An AC-feasible start places the
iterate in the second regime from the first iteration: the (θ,φ)
machinery never has to recover feasibility, only to decrease φ — the
regime where every component (inertia-controlled LDLᵀ step, SOC,
domination history) is operating at design point.  The DC-OPF→ACPF
cascade fails on stressed grids for the dual reason: the DC dispatch is
AC-inconsistent (reactive/voltage-wise far from any AC solution), so
Newton–Raphson diverges *from it* — ACPF correction must be applied to
the default start, not to the DC dispatch.

**Boundary of the result (honest).**  Feasible-but-off-center starts are
not uniformly better: case1354 slows 88 → 330 iterations (centrality at
init beats exact feasibility on well-posed cases — the same reason Ipopt
initializes by least squares + bound pushes), case6468rte lands in a bad
basin (obj −38%), case6495rte fails outright.  The option is therefore
opt-in, indicated for large stressed grids whose default-start trajectory
fails; case2869/9241pegase remain open even with it (their failure is the
undershoot mode, not the start), where continuation on the stress
parameter or full elastic restoration is the documented next mechanism.

---

## 12. Phase I → Phase II as objective homotopy: the theory-guided mapping

**Question.**  For hard (stressed, near-infeasible) OPF instances: we have
an AC solution meeting power balance but violating some inequalities — the
start is not in the feasible interior.  Can the IPM be split into Phase I
(find a feasible point) and Phase II (improve its quality)?  The simplex
method does exactly this, but the LP feasible set is convex; for nonconvex
OPF the Phase-I→II *mapping* needs theory, not trial and error.

**Why the hard switch fails.**  Phase I as the zero-cost problem
(min 0 s.t. g = 0, h + z = 0, z ≥ 0) converges fast (24–50 iterations on
the PEGASE cases — the barrier alone drives it).  But switching the
objective to full cost in one step breaks the iterate's *centrality*:
the Phase-I duals satisfy stationarity of a Lagrangian with no economic
gradient, so the costed KKT rejects every step (measured: 639 iterations
with zero filter accepts, dual residual pinned at 1e-2), while resetting
the duals loses the basin (trajectory dives to the infeasible
low-objective region).  Warm-start theory says why: a primal-dual IPM
start must be *well-centered*, and perturbations are absorbed in few
steps only from such starts (Gondzio–Grothey, SIAM J. Optim. 13, 2003;
Chen–Goulart–Jones 2025, arXiv:2512.00693 — centrality, not residual
minimization, is the invariant to preserve).

**The mapping.**  Follow the KKT path of the objective homotopy
`P(t): min t·f(x) s.t. g = 0, h + z = 0, z ≥ 0` for `t : 0 → 1`.
Under LICQ + SOSC + strict complementarity the path `w(t)` is C¹
(implicit-function theorem on the perturbed KKT — the same argument that
gives the central path's existence, cf. Armand–Benoist–Orban Lemma 3.3).
Each continuation step is a full IPM solve (corrector) warm-started from
the previous path point with the *complete* primal-dual state — the path
duals are by construction consistent with `P(t)`'s stationarity, so the
hard-switch inconsistency never occurs.  The step size adapts to the
local convergence radius: a step converging in ≤ 20 iterations doubles
Δt, a failure halves it (Todd's metric viewpoint on homotopy parameter
adjustment; Birgin–Krejić–Martínez inexact-restoration criteria, which
also identify turning points of the path with local violation minimizers
— our honest "minimal violation" report when Δt collapses).
Continuation-on-constraints for stressed OPF is the same idea in the
constraint direction (Park–Glista–Lavaei–Sojoudi, homotopy for
post-contingency OPF).

**Measured path.**  case2869pegase: obj(t) tracks `t·f*` almost exactly
(26 800 at t = 0.2 → 133 999 at t = 1), each step converging; the t = 1
endpoint certifies at **obj = 133 999 — the exact PGLIB reference
optimum** — in 5 further iterations (vs baseline: filter-fail at obj
92 926; vs hard-switch two-phase: +0.74% uncertified).  case1354 and
case1888rte also converge certified.

**Two finishing mechanisms (also theory-standard).**

1. *Objective-stagnation early stop.*  Hard trajectories reach the
   endpoint long before any tolerance fires — the tail is restoration
   crawl whose steps carry zero stationarity rhs, so the dual residual
   cannot improve through them.  When feasibility (< 1e-4) and
   complementarity (< comp_tol) are met and |Δobj| ≤ 1e-6(1+|obj|) over
   20 accepted iterations, stop and certify by polish.  Cuts ~80% of
   tail time; gated by feasibility so a healthy tail (where obj
   stagnates while θ is still reduced) is never cut.

2. *Dual least-squares polish.*  At the endpoint, re-estimate multipliers
   from the exact stationarity LS `min_{λ,μ} ½‖f̃∇f + Jgᵀλ + Jhᵀμ‖²`,
   whose KKT system is `[I Aᵀ; A 0]` with A = [Jg; Jh] — one solve with
   the same LDLᵀ machinery (inactive-row multipliers clipped to μ ≥ 0;
   solution certification per the sparse-LS literature).  This is the
   standard finishing step (Ipopt/MIPS multiplier estimates) and turns
   feasible-but-dual-floored endpoints into certified KKT points with an
   honest status (`converged (dual least-squares certified)`).

---

## 13. Davidenko tangent prediction: the step-efficiency theory

**Problem.**  The §12 objective homotopy is the correct Phase I→II mapping,
but its plain form is expensive: on case9241pegase each continuation step
costs 270–460 corrector iterations (a near-cold re-solve per step) and the
path dies at t ≈ 0.55 without help.  The efficiency question has a clean
theoretical answer: follow the path with its *tangent* instead of jumping
blindly.

**Derivation.**  The KKT path `w(t) = (x, z, λ, μ)` of
`P(t): min t·f(x) s.t. g = 0, h + z = 0, z ≥ 0` satisfies `F(w(t), t) = 0`.
Differentiating in `t` gives the Davidenko equation

```
∂F/∂w · w′(t) = −∂F/∂t = −(∇f, 0, 0, 0)ᵀ
```

and `∂F/∂w` is exactly the IPM's augmented KKT, so **one back-solve with
the existing LDLᵀ factorization** (analyze-once, inertia-controlled)
yields the whole tangent, with `dz/dt = −Jh·dx/dt` recovered analytically:

```
[ Lxx   Jgᵀ   Jhᵀ  ] [dx/dt]   [−∇f/t]
[ Jg     0     0   ] [dλ/dt] = [ 0   ]
[ Jh     0   −ZM⁻¹ ] [dμ/dt]   [ 0   ]
```

(∇f is the t-scaled objective gradient; ∇f/t is the full cost gradient
entering the stationarity at rate 1.)  The predictor
`w_pred = w(t) + Δt·w′(t)` is first-order accurate (error O(Δt²)), and —
crucially — its duals are *path-consistent* by construction: the
hard-switch inconsistency that killed the naive two-phase never occurs.

**Step-size theory (parameter-metric trust radius).**  A predicted step is
valid only inside the corrector's convergence radius.  Todd's induced
parameter metric maps the solution-space distance to a parameter-space
step; the practical form is a component trust radius on the extrapolation
`h ≤ κ/‖dx/dt‖∞` — measured here to be load-bearing: uncapped steps
overshoot into infeasibility (viol ~ 1–10 at iteration 1), while
κ = 0.05 keeps every predicted start in the corrector's basin.  The warm
mapper additionally clamps predicted points into the bounds interior.
Birgin–Krejić–Martínez's inexact-restoration criteria supply the
accept/reject test and the turning-point diagnosis (Δt collapse ⇒ honest
minimal-violation report).

**Measured.**  case9241pegase — previously: filter-fail in every variant;
plain homotopy without predictor dies at t ≈ 0.55 (obj 288 970,
off-path).  With tangent prediction + centrality recovery + dual polish:
**full path to t = 1, converged at obj = 315 873** (−0.013% vs the PGLIB
reference 315 913), 25 continuation steps, the final solve certifying in
6 further iterations.  Steps near t = 0 drop from ~300 to 28–78 corrector
iterations; case2869pegase converges at the exact reference 133 999
(107 s via `ACOPFOptions::objective_homotopy`).

---

## 14. Per-iteration re-equilibration is a noise source (the IEEE24 debug)

**Observed.**  A hybrid AC/DC OPF (IEEE24-3area-expanded: 24 AC buses, 8 DC
buses, 8 VSC + 2 DCDC converters, DC line resistances r ≈ 0.004 pu ⇒
conductances g ≈ 250 pu) fails to converge: primal feasibility oscillates
in a 3–8e-3 band for 640 iterations with stationarity already at 1e-7 —
while the plain power flow converges in 5 iterations and Ipopt drives the
DC-balance residual to 1.5e-4.  The feasibility valley of a network with
g ≈ 250 pu is ~1/250 wide in voltage space; landing in it requires the
Newton direction to be metrically stable across iterations.

**Root cause (measured).**  The Ruiz equilibration `D` was recomputed
*every* iteration from the current KKT.  As the barrier pairs evolve, the
`−z/μ` block sweeps ratios of ~1e±10, so the row norms — and thus `D` —
swing by orders of magnitude between consecutive factorizations.  On a
stiff KKT the solve is performed as `(D K D)y = D b`, and the effective
Newton direction `x = D y` therefore *jitters with D*: consecutive
directions differ by more than the valley width, and the trajectory
limit-cycles above the tolerance instead of landing in it.  All
eliminated alternatives: δ_C·‖λ‖ floor (λ_DC ≈ 21.5, δ_C sweep 1e-8–1e-12
no effect), linear-solver brand (MUMPS/UMFPACK/KLU/Eigen identical),
genuine infeasibility (Ipopt closes to 1.5e-4), storage pinned by design.

**Fix (measured).**  Compute the equilibration **once per solve** and
freeze it (standard IPM practice — Ipopt scales from the NLP once).
IEEE24-expanded: 640-iteration stall → **36 iterations to certified
convergence** (viol 6.1e-6, stat 2.1e-7, 0.02 s).  Freeze is the default;
`HACDCPF_OPF_RESCALE_PER_ITER=1` restores the old rebuild for A/B.
Side benefits: no per-iteration Ruiz sweeps (~5×nnz per iteration saved —
case9241 homotopy 721 → 606 s), and several trajectories improve
(case1888rte 168 → 89 iterations).

---

## 15. Forrest-Tomlin updates require triangular-solve intermediates

For a simplex exchange `B'=BE_p(v)`, `v=B^-1 A_q` proves the product-form
identity, but it is not by itself the data layout consumed by HFactor's sparse
Forrest-Tomlin implementation. The FT column pack is the state after the L
solve and before the U solve; the row pack is the state after `U^-T` and before
`L^-T`. Repacking the final FTRAN/BTRAN vectors therefore constructs a
different update even though the final pivot value is correct.

The native LP factor wrapper captures both intermediates during their original
solves. HFactor's INVERT also permutes basis positions into pivot-row order, so
the leaving position and FTRAN result are mapped into that coordinate space;
the BTRAN result remains in physical row space. Captures carry the factor
generation. A same-basis reinversion regenerates stale packs from their saved
right-hand sides before committing the selected exchange.

This contract is tested from non-diagonal, permuted bases through ten column
exchanges, with every FTRAN and BTRAN compared against both fresh HFactor and
dense LU. Diagonal crash bases alone are insufficient because they collapse
the caller-basis, pivot-row, and physical-row mappings into the same indices.
The private HFactor port also retains every nonzero produced by factorization,
triangular solves, and FT replay. Upstream `kHighsTiny` dropping remains in the
embedded HiGHS target, but is incompatible with checking native solves against
the true explicit basis matrix at an unchanged backward-error bound.

---

## 16. Terminal active-set KKT crossover for nonlinear programs

A finite-barrier point can satisfy the requested complementarity tolerance while
remaining (O(\mu)) from an active inequality boundary. NativeIPM therefore
attempts an optional active-set Newton crossover when an ordinary filter solve
converges. Under strict complementarity, active slacks are (O(\mu)) with
multipliers bounded away from zero, whereas inactive multipliers are (O(\mu))
with slacks bounded away from zero. The implementation uses a
(\sqrt{\mu})-scale separator and solves the limiting equality-constrained KKT
system (Nocedal and Wright, 2006, Sections 16.3 and 19.6).

Candidate rows are ordered by central-path confidence. Structural row rank is
selected by maximum matching on the sparse Jacobian bipartite graph (Duff,
1981), after which sparse LDLT inertia and solve-residual checks provide the
numerical certificate. At most eight dual-sign repairs and eight active-set
Newton steps are attempted, so degeneracy cannot make this optional crossover
unbounded. Multiplier projection alone is never sufficient: at least one
active-set Newton step must be accepted, and the full unperturbed primal, dual,
and complementarity residuals must satisfy the user tolerances. If any rank,
inertia, sign, line-search, or final KKT check fails, the already converged
barrier point is retained (Fletcher, 1987, Section 10.3).

`kMinPositive` remains a denominator-regularization threshold; trial slacks and
multipliers are accepted whenever they are finite and strictly positive, even
below that threshold. After a rejected crossover, the filter path may use one
additional barrier decade,
`max(mu_min, 0.01 * tol_complementarity)`, consistent with the first-order
central-path displacement in (\mu). The regression with one active and 64
inactive inequalities at scale (10^4) verifies that the solver reaches the
limiting active boundary and reports `Converged after active-set KKT polish`.

---

## 17. Complete Ipopt warm starts require row-order and bound-dual mapping

The public NLP multiplier order is
`[nonlinear inequalities | equalities]`, whereas the Ipopt TNLP starting-point
callback expects `[equalities | nonlinear inequalities]`. When
`NLPSolverOptions::primal_dual_warm_start` is enabled, `CallbackTNLP` performs
this permutation without changing signs, because both formulations use
`f + lambda' g + mu' h` for `h(x) <= 0` and `mu >= 0`. It also passes the
variable lower- and upper-bound multipliers `z_L` and `z_U`. Ipopt reconstructs
its internal inequality slacks; Native IPM slack vectors are not part of the
TNLP starting-point contract (Wachter and Biegler, 2006, Section 3.1; Ipopt
`TNLP::get_starting_point`).

The adapter rejects incomplete dimensions, nonfinite values, negative
inequality multipliers, and negative bound multipliers. With a valid complete
start it enables `warm_start_init_point` and uses a common `1e-8` bound, slack,
and multiplier push. Unit tests return and reapply a KKT point containing both
an equality and an active nonlinear inequality, so the two row conventions are
checked in both directions.

The HySim four-feeder parametric OPF benchmark holds the graph-reduced model,
generated row set, `+0.1%` load perturbation, Ipopt options, and five alternating
repeats fixed. Median primal-dual/primal-only times are `2.064/3.522`,
`4.400/7.178`, `6.451/12.664`, and `274.799/743.846 ms` on
H13/H34/H123/H8500. The corresponding speedups are `1.707x`, `1.632x`,
`1.963x`, and `2.707x`, with a `1.961x` geometric mean. This is dirty-worktree
Apple M4 Max evidence, not a clean pinned-release result.

### Windows integration verification boundary

The integration of Windows `75c6e192` and main `5eac6be0` retains the Windows
strict caller-coordinate gates and exact-first KKT regularization. Augmented
congruence solves `T K T z=T rhs` and recovers `T z`; optional active-set polish
also passes the strict gate before replacing a certified barrier point.
The first imported-test failures came from default-policy assumptions: Windows
automatic `mu_init=0` cannot be used to construct positive supplied duals, and
near-feasible warm-start tests must state their intended 0.01/0.5 audit policies.
The fixtures now state those policies, leaving production defaults and final
accuracy gates unchanged. Windows also counts one full-derivative certificate
in the one-variable quadratic diagnostic. Final IPM: 54 cases / 368 assertions
passed. Full commands, parent revisions, errors and remaining scope are in
[the maintained integration record](../manual/09-testing-benchmarks.md#windowsmain-integration-validation).

Downstream mismatch: the all-regressions-pass prediction does not hold for two
Simulation OPF cases. The full SDK compiles and resilience/market suites pass,
but the GUI Auto showcase reaches Ipopt's iteration limit and Native
graph-reduced primal-dual transport reports accepted-step collapse. Both pass
on pure main in a paired rerun. The integration retains materially different
Windows strict-bound/termination and Native globalization policies; causality
is not isolated, so no mathematical threshold or solver policy is changed in
response. Simulation's dependency pin remains pure main. These residuals,
commands and failed assertions are recorded in the maintained integration
record, and must be resolved before declaring downstream OPF compatibility.

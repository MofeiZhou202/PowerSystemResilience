# Beating HiGHS: Theory and Engineering Roadmap for the Native MILP Solver

**Date:** 2026-07-28
**Scope:** what it takes for the native stack in `src/` (dual simplex + SparseBasis LU, IPM-LP, PDLP, native B&C) to *fundamentally* outperform HiGHS on MILPs, with full theoretical derivations and a phased engineering plan. Numerical robustness inside branch-and-cut is treated as a first-class design axis, not an afterthought.
**Grounding:** all "current state" claims come from `docs/native_vs_highs_milp_evaluation.md` (2026-07-20 rounds 1–4), `docs/lp_first_order_methods_review.md` (2026-07-28), and a code inventory of `src/engine/` performed for this document.

---

## 0. TL;DR

1. **Historical result, not a general conclusion.** One 118-bus SCUC run reported a smaller final gap for native branch-and-cut on the HiGHS LP kernel than for the selected HiGHS configuration. That single-domain observation does not establish a general advantage, and the native flow-cover family formerly listed here was removed because its validity conditions were not enforced.
2. **The decisive battle is the simplex kernel.** Native per-pivot cost is ~3 ms at 27–35k rows vs HiGHS ~µs (**~100–1000× per node-LP**); 8/10 warm starts die in UMFPACK-singular warm-drifted bases; Phase I fails on the presolved 118-bus root. No amount of cut/branching tuning can amortize a 100× slower node LP: with ~50–80% of B&C time in LP re-solves, the kernel bounds everything (§1).
3. **The fundamental fixes are algorithmic, not tuning:** (a) a simplex-native factorization with rank-repair and Forrest–Tomlin-class updates instead of general-purpose UMFPACK+eta (§2.2, §4-P1); (b) hyper-sparse FTRAN/BTRAN — SCUC LPs are precisely the hyper-sparse family HiGHS was built for (§2.2.4); (c) bound-flipping dual ratio test + dual steepest edge done with stability guards (§2.1); (d) anti-degeneracy machinery (perturbation + EXPAND) because SCUC is maximally degenerate (§2.3); (e) a numerics contract — one tolerance architecture with dominance invariants, safe cut generation, and audits at every trust boundary (§5).
4. **Realistic win conditions are staged** (§6): first beat HiGHS end-to-end natively on the SCUC family (achievable: the orchestration already wins when the kernel is borrowed), then reach ≤1.3× shifted-geometric-mean on a MIPLIB-2017 subset, then attack parity/beyond with the asymmetric weapons HiGHS lacks: structure-aware SCUC cuts/branching, L2O-guided configuration (the `l2o/` framework), a concurrent root portfolio (simplex ∥ IPM ∥ PDLP), and safe-numerics honesty as a feature (native already rejects the false optima HiGHS silently returns at 10⁶ scaling).

Reading map: §1 performance model → §2 theory with derivations → §3 gap analysis vs HiGHS → §4 phased engineering plan → §5 numerical playbook for B&C → §6 win conditions → §7 references.

---

## 1. A performance model of branch and cut

### 1.1 Where the time goes

Let the tree explore $N$ nodes. Total wall time decomposes as

$$T \;=\; t_{\text{presolve}} + t_{\text{root}} + \sum_{k=1}^{N} \big(t^{(k)}_{\text{LP}} + t^{(k)}_{\text{prop}} + t^{(k)}_{\text{sep}} + t^{(k)}_{\text{heur}} + t^{(k)}_{\text{branch}}\big).$$

Across solvers and benchmarks, node LP re-solves take 50–80% of $T$ (Bixby; Achterberg's SCIP accounting). There are exactly two multiplicative levers:

- **Reduce $N$** — better dual bounds (presolve, cuts, tighter relaxations), better primal bounds (heuristics, node ordering), better branching. Tree size responds *exponentially* to per-node gap closure: in the abstract branching model of Le Bodic–Nemhauser, a variable that closes $(g^-, g^+)$ per branch yields $N \approx 2^{G/\bar g}$ for remaining gap $G$, so doubling average per-node gain roughly *squares-roots* the tree.
- **Reduce per-node cost** — warm-started dual simplex measured in *a few pivots × cheap pivots*. HiGHS achieves ~0.4 ms/node-LP on 118-bus-class SCUC; the native kernel currently needs 0.1–5 s.

The product structure means a solver that is 2× better on nodes but 100× worse per node loses by 50×. This is exactly the measured state: the native tree explored 10 LPs in ~150 s where HiGHS did ~720k LPs in 300 s (`native_vs_highs_milp_evaluation.md` §5.7). **Hence the strategic ordering of §4: kernel first, tree second, domain/learning third.**

### 1.2 What "beat HiGHS" must mean operationally

A performance claim is meaningful only under a fixed protocol (Mittelmann-style):

- **Metric:** shifted geometric mean (SGM) of wall time, shift 10 s, timeout counted at limit (or PAR-2); plus performance profiles. Never arithmetic means; never per-instance cherry-picks.
- **Correctness gate:** a run counts only if the returned incumbent passes an *independent original-space audit* (row/bound/integrality violations ≤ tolerances; the machinery exists — `native_kernel_comparison --check`) and claimed optimality is consistent with the reference bound. HiGHS's false "Optimal" on all 5 NETLIB problems at 10⁶ scaling shows why the gate must apply to the *baseline* too.
- **Instance sets:** (a) the SCUC family (6/39/118-bus × 4/24 T, plus harder market-clearing variants) — the domain target; (b) MIPLIB 2017 benchmark subset — the generality target; (c) adversarial scaling probes — the robustness target.
- **Parity of resources:** same thread count, same time limits; separately report 1-thread and n-thread.

---

## 2. Theory

Notation: LP in computational (bounded, equality) form

$$\min\; c^\top x \quad \text{s.t.}\quad Ax = b,\;\; \ell \le x \le u,\qquad A \in \mathbb{R}^{m\times n}\ (\text{slacks included, so } n \ge m),$$

basis $B$ (an ordered index set with $A_B$ nonsingular), nonbasic set $\mathcal N$ with each $j \in \mathcal N$ at $\ell_j$ or $u_j$ (or free-at-zero). Primal values $x_B = A_B^{-1}(b - A_{\mathcal N} x_{\mathcal N})$; duals $y^\top = c_B^\top A_B^{-1}$; reduced costs $d_j = c_j - y^\top a_j$.

### 2.1 The bounded-variable dual simplex — why it owns the tree

#### 2.1.1 Dual feasibility is invariant under bound changes (the warm-start theorem)

Dual feasibility of $(B, \text{bound-status})$ for a minimization problem is

$$d_j \ge 0 \;\; \forall j \text{ at } \ell_j, \qquad d_j \le 0 \;\; \forall j \text{ at } u_j, \qquad d_j = 0 \;\;\forall j \text{ free},$$

and — crucially — **does not involve $b$, $\ell$, or $u$**. Branching changes only bounds ($x_j \le \lfloor x_j^* \rfloor$ or $x_j \ge \lceil x_j^* \rceil$); adding a cut row $\alpha^\top x \le \beta$ adds one slack column which enters the basis with $d_{\text{slack}} = 0$. Therefore:

> **Theorem (warm start).** The optimal basis of the parent node LP remains *dual feasible* for every child LP obtained by bound tightening and/or cut addition. Dual simplex can resume from it directly; only primal feasibility ($\ell \le x_B \le u$) is violated, typically in few components.

Empirically, a child LP re-optimizes in $O(1)$–$O(10^2)$ pivots instead of $O(m)$ cold pivots. This single fact is why every competitive MILP solver uses dual simplex in the tree and why node-LP speed = pivot count × pivot cost is the governing product. (It is also why the round-3 finding — 8/10 warm attempts failing and falling back to cold solves — is catastrophic: each failure converts an $O(10)$-pivot solve into an $O(10^5)$-pivot one.)

#### 2.1.2 One dual pivot, and the ratio test derived

Choose a leaving row $r$ whose basic variable violates a bound; say $x_{B_r} > u_{B_r}$ (the case $< \ell$ is symmetric), primal infeasibility $\delta = x_{B_r} - u_{B_r} > 0$. Compute the **BTRAN** direction and pivot row

$$\rho = A_B^{-\top} e_r, \qquad \alpha_j = \rho^\top a_j \;\; (j \in \mathcal N).$$

Increasing the dual multiplier on row $r$ by $\theta \ge 0$ (in the direction that pushes $x_{B_r}$ down) changes reduced costs as $d_j \leftarrow d_j - \theta\,\alpha_j$ (sign convention: leaving-at-upper). The **admissible blocking set** is the nonbasic $j$ whose $d_j$ would cross zero:

$$J = \{\, j \text{ at } \ell_j : \alpha_j < 0 \,\} \cup \{\, j \text{ at } u_j : \alpha_j > 0 \,\},\qquad \theta_j = d_j / \alpha_j \ \ (\ge 0).$$

The dual objective as a function of $\theta$ is **piecewise linear and concave** with initial slope $\delta$; each breakpoint $\theta_j$ passed either terminates the step (variable $j$ enters) or — if $x_j$ is **boxed** ($u_j - \ell_j < \infty$) — the variable can **flip to its opposite bound** and the slope decreases by

$$\Delta\text{slope}_j = (u_j - \ell_j)\,|\alpha_j|.$$

**Bound-flipping ratio test (BFRT / "long step", Fourer; Maros; Koberstein).** Sort/scan breakpoints in increasing $\theta$; keep flipping boxed blockers while the slope stays positive; pivot on the breakpoint where the slope would turn non-positive. All flipped variables change bounds simultaneously; $x_B$ is updated by one extra FTRAN of the aggregated flip column $A_F \Delta x_F$. Payoff: on problems with many boxed variables (SCUC commitment/dispatch variables are almost all boxed) a single dual iteration does the work of dozens, and it *skips over degenerate vertices* ($\theta = 0$ breakpoints) instead of pivoting through them.

**Harris two-pass ratio test (numerical safety).** Exact ratio tests select breakpoints determined by tiny $|\alpha_j|$ — numerically suicidal pivots. Harris' fix: pass 1 computes the relaxed step $\theta_{\max} = \min_j (d_j + \varepsilon_d)/\alpha_j$ over $J$ (tolerance-expanded); pass 2 picks, among all $j$ with $\theta_j \le \theta_{\max}$, the one with **largest $|\alpha_j|$** (best stability). The admitted dual infeasibility is bounded by $\varepsilon_d$ and is repaired by shift-removal later (§2.3). This is the correct home for the pivot-quality battle of eval-doc round 4 — the guard belongs *inside the ratio test's candidate choice* (prefer large $|\alpha_j|$ among near-tied breakpoints), not as a post-hoc absolute floor that starves Phase I.

#### 2.1.3 Dual steepest edge (DSE) pricing

Choosing *which* infeasible row leaves determines convergence speed. Dantzig pricing (max $\delta_r$) is scale-dependent; **dual steepest edge** (Forrest–Goldfarb) maximizes the infeasibility *per unit length of the actual dual step*:

$$r \in \arg\max_i \frac{\delta_i^2}{w_i}, \qquad w_i = \lVert A_B^{-\top} e_i \rVert_2^2.$$

Exact recomputation is one BTRAN per row — unaffordable; the Forrest–Goldfarb update maintains all weights after a pivot (entering $q$, leaving row $r$) using quantities already computed: with $\hat\alpha = A_B^{-1} a_q$ (the FTRAN column), $\rho = A_B^{-\top} e_r$, and one extra FTRAN $\tau = A_B^{-1} \rho$:

$$\bar w_r = \frac{w_r}{\hat\alpha_r^2}, \qquad
\bar w_i = w_i \;-\; 2\,\frac{\hat\alpha_i}{\hat\alpha_r}\,\tau_i \;+\; \left(\frac{\hat\alpha_i}{\hat\alpha_r}\right)^{2} w_r \quad (i \ne r),$$

clamped below by a small positive floor (drift in $w$ is a *cheap curvature-error monitor*: a weight going negative is an early singularity/instability alarm — directly useful against the round-3 warm-basis failures). Cost: one extra FTRAN per pivot, repaid many times over in pivot-count reduction on degenerate LPs. HiGHS uses exactly this scheme.

#### 2.1.4 Dual Phase I

When the warm basis is *dual infeasible* (cold starts; after cost changes), competitive codes avoid big-M and use either (a) the **subproblem approach** (minimize total dual infeasibility $\sum_j \max(-\sigma_j d_j, 0)$, piecewise-linear, solvable by the same dual iteration with modified bounds — Koberstein's thesis, ch. 6) or (b) **cost shifting**: perturb/shift individual $c_j$ just enough to make $d_j$ feasible, run Phase II, then remove shifts and clean up (HiGHS: `HEkkDual` shifted costs + `dual_cleanup_resolve`; the repo's A/B port of this exists and lost — revisit *after* the factorization work, because Phase-I pivot sequences on the presolved 118-bus root are exactly where the current kernel dies; see §6 item 1 of the eval doc).

RHS/bounds must never be perturbed for Phase I on equality-heavy models — the eval doc's round-1 bug (perturbing RHS made tightly-balanced power-balance rows *genuinely infeasible*, producing false infeasibility certificates) is the canonical counterexample and is now a playbook rule (§5, N5).

### 2.2 The factorization engine — where the 100× lives

Everything in §2.1 executes through four kernels: FTRAN ($A_B^{-1} a$), BTRAN ($A_B^{-\top} e$), factorize ($A_B = LU$), update (replace one column of $A_B$). Their cost and stability determine pivot cost; this subsection is therefore the highest-leverage theory in the whole document.

#### 2.2.1 LU with Markowitz pivoting and threshold partial pivoting

Factor $A_B = LU$ choosing pivots to balance sparsity and stability: among candidates $(i,j)$ with

$$|u_{ij}| \ \ge\ \tau \cdot \max_k |u_{kj}| \qquad (\text{threshold } \tau \in [0.01, 0.99],\ \text{typical } 0.1),$$

pick the one minimizing the **Markowitz merit** $(r_i - 1)(c_j - 1)$ ($r_i, c_j$ = remaining row/column counts). Small $\tau$ → less fill-in, larger growth factor $\rho = \max |u_{ij}| / \max |a_{ij}|$; backward error is $O(\rho\, \varepsilon_{\text{mach}})$, so $\tau$ is the stability dial. **Escalation on failure** (repo already does this: 0.1 → 1.0 → KLU) is correct; what is missing is *rank repair* (§2.2.3).

#### 2.2.2 Product-form vs Forrest–Tomlin updates

After a pivot, $\bar A_B = A_B + (a_q - A_B e_r) e_r^\top$. Two classical ways to avoid refactorizing:

- **Product-form (eta) update:** $\bar A_B^{-1} = E^{-1} A_B^{-1}$ with eta matrix $E = I + (\hat\alpha - e_r) e_r^\top$, $\hat\alpha = A_B^{-1} a_q$. Cheap to create (the FTRAN column is already computed), but FTRAN/BTRAN cost grows by $O(\text{nnz}(\hat\alpha))$ *per accumulated eta*, and — decisive for stability — the eta pivot is $\hat\alpha_r$, whatever the ratio test produced. Long eta chains on degenerate problems compound error multiplicatively. This is the repo's current scheme on top of UMFPACK.
- **Forrest–Tomlin (FT) update:** work on the factors instead. Solve $Lv = a_q$; replacing column $r$ of $U$ by $v$ gives an upper-triangular-plus-spike matrix; symmetrically permute position $r$ to the last position (cyclic shift) so the spike becomes a **row** spike; eliminate that row against the pivots of $U$, storing the multipliers as one **row eta** $R$:
  $$\bar U = R^{-1} P U' P^\top,\qquad \bar A_B = L \,(\text{accumulated } R\text{s})\, \bar U \ (\text{+ permutations}).$$
  Cost per update ~ $O(\text{nnz}(\text{spike}))$; FTRAN/BTRAN stay near-factorization cost; the update pivot element is monitored (Suhl–Suhl variant tightens this) and **a too-small update pivot triggers refactorization** — a built-in stability sentinel the eta scheme lacks.

HiGHS' `HFactor` implements FT with hyper-sparse solves. The vendored port in `src/engine/kernel/linear_algebra/highs_factor/` is the intended asset here; eval-doc round 4 found its `updateFT` port broken (incomplete row-eta content, Sherman–Morrison-verified divergence at the 2nd consecutive update on a 200×200 reproducer) — *fixing that port, or using fresh-HFactor-build + native eta as already wired, is the single highest-value kernel task in §4-P1.*

#### 2.2.3 Rank repair: the missing capability behind the warm-start failures

The round-3/4 diagnosis: warm-hint bases (parent basis + child bounds) drift to **exact singularity** within ≤8 pivots; UMFPACK's numeric phase rejects them; there is no repair path, so the node burns a crash-repair reopt and a cold solve (~5 s each). The fix is standard in simplex codes:

> **Basis repair.** During factorization, detect deficient pivots (no admissible pivot in a row/column). For each deficient row $i$, replace the offending basic column with the **logical (slack) column $e_i$** of that row. The repaired matrix is nonsingular by construction (unit pivots), the repaired basis is a *valid* simplex basis, and the ejected columns become nonbasic at bound — introducing bounded primal/dual infeasibility that the same warm dual simplex removes in a few pivots.

This requires the factorization to *report* deficient positions — which `HFactor` does natively (`build()` returns rank-deficiency info and performs exactly this logical substitution), and UMFPACK does not expose. That asymmetry, more than raw speed, is why "UMFPACK+eta" is the wrong architecture for a simplex kernel and why eval-round-4's whack-a-mole repair attempts on top of UMFPACK churned: repair must happen *inside* the factorization with its pivot-level information, not by post-hoc column swapping. (The observed chronic borderline bases are themselves a symptom of §2.1.2-style pivot admission choosing structurally dependent columns on degenerate vertices; ratio-test candidate preference + DSE weight alarms attack the cause, basis repair removes the cliff.)

#### 2.2.4 Hyper-sparsity: the reason HiGHS is fast on exactly this problem family

Hall–McKinnon's observation: on LPs from network-like structures (unit commitment explicitly among them), the FTRAN/BTRAN results are themselves sparse — often <1% nonzeros. Then computing $A_B^{-1}a$ by dense triangular solves costs $O(m + \text{nnz}(L))$, while a **graph-driven solve** (Gilbert–Peierls) costs $O(\text{nnz(result)} + \text{flops on the reachable set})$:

1. Symbolic phase: DFS from the nonzero pattern of $a$ over the DAG of $L$ to find the reachable set = exact nonzero pattern of the solution.
2. Numeric phase: triangular elimination touching *only* that set.

With <1% dense results this is a 100×-class speedup **per solve** — matching, not coincidentally, the observed per-pivot gap (~3 ms native vs ~µs HiGHS at 27–35k rows). The same principle applies to the pivot row (BTRAN + row-wise A access), the eta applications, and PRICE (row · $A_{\mathcal N}$, needing a row-wise/CSR copy of $A$). Implementation discipline: maintain dense/sparse/hyper representation switching per vector with density thresholds (~10%/1%), and an *indexed nonzero list* alongside the dense array so results never need $O(m)$ scans. **This is a rewrite-level feature of the solve path, not a tweak** — it is P1's core (§4).

#### 2.2.5 Error growth, refactorization control, and κ monitoring

Backward error of a solve through $k$ accumulated updates grows roughly like $O(\varepsilon_{\text{mach}} \cdot \rho \cdot \kappa(A_B) \cdot g(k))$ with $g$ increasing in chain length; competitive codes bound it by:

- **Scheduled refactorization** every $K$ updates ($K \approx 100$ classical; adaptive up to several hundred when updates are cheap/stable), *plus*
- **Triggered refactorization** on any of: too-small FT update pivot; growth of $\lVert \text{spike} \rVert$; FTRAN residual spot-checks $\lVert A_B \hat\alpha - a_q \rVert > \varepsilon_r \lVert a_q \rVert$ (cheap: one SpMV against the *original* columns); DSE weight negativity; primal value drift on bound-flip updates.
- **Condition estimation:** a $\kappa_1(A_B)$ estimate via Hager–Higham (a few FTRAN/BTRANs) exposed in `[LP-STATS]`; large κ downgrades cut generation trust (§2.5.6) and tightens the Harris tolerance — the "numerical emergency mode" pattern (Gurobi's public description; HiGHS similar internally).

The **optimal refactorization interval** balances marginal update cost against amortized refactor cost: refactor when $\sum_{\text{updates}} \Delta t_{\text{solve}} \ge t_{\text{refactor}}$ — self-tuning with two counters, no constants to guess.

### 2.3 Degeneracy — SCUC's defining pathology

Unit-commitment LPs are extreme on both degeneracy axes: **primal** (thousands of binding capacity/ramp/balance equalities → basic variables exactly at bounds → zero-length ratio-test steps) and **dual** (identical generators and repeated time periods → massed duplicate reduced costs → ties everywhere in pricing). Consequences, all observed in the eval doc: stalling/cycling risk, warm re-solves that wander (hundreds of pivots for a single bound change), Phase I collapse at scale, vanishing strong-branching signals (§2.6).

The standard three-layer defense, derived:

1. **Cost perturbation (dual side).** Replace $c_j \leftarrow c_j + \xi_j$, $\xi_j$ small, random, *magnitude-aware* ($\xi_j \propto \varepsilon_p (1 + |c_j|)$, deterministic seed), applied once at (re)start on degenerate-prone solves. Ties in $d_j$ break generically; the perturbed LP's optimal basis is optimal for a nearby problem. **Removal contract:** restore exact $c$, recompute $d$, and run a cleanup dual (or primal) re-solve — bounded by few pivots since the basis is near-optimal. Never perturb equality RHS (playbook N5).
2. **EXPAND (Gill–Murray–Saunders–Wright).** Maintain a *working* feasibility tolerance $\delta_k$ that grows by a tiny increment each iteration ($\delta_{k+1} = \delta_k + \tau_{\text{inc}}$, reset at refactorization with variables snapped back to true bounds). Because the acceptable step at each ratio test is measured against $\delta_k$, **every iteration can take a strictly positive step**, which (in exact arithmetic on the perturbed tolerances) forbids cycling without Bland-rule pessimism. Cheap and composable with Harris.
3. **BFRT long steps (§2.1.2)** hop over $\theta = 0$ breakpoints entirely — on boxed-variable-heavy SCUC this is the single biggest degeneracy weapon.

These three are *jointly* the alternative to the rejected absolute-pivot-floor experiments of eval round 4: instead of refusing small pivots (which Phase I legitimately needs), make small-pivot situations rare (BFRT + perturbation) and harmless (Harris stability preference + EXPAND + repair).

### 2.4 Interior point for the root, and the crossover bridge

The tree needs dual simplex; the **root** of a large SCUC LP (and Benders masters, market-clearing LPs) is often better served by IPM, then converted to a basis. This is also the current repo direction (CHOLMOD LDLᵀ augmented-KKT commits).

#### 2.4.1 Newton system and its conditioning cliff

Primal-dual path following solves, at each $\mu$, the perturbed KKT system $Ax = b$, $A^\top y + s = c$ (plus box duals), $XSe = \mu e$. Eliminating $\Delta s$ gives the **augmented system**

$$\begin{pmatrix} -D^{-2} & A^\top \\ A & 0\end{pmatrix}\begin{pmatrix}\Delta x\\ \Delta y\end{pmatrix} = \begin{pmatrix} r_d \\ r_p \end{pmatrix},\qquad D^2 = X S^{-1},$$

and eliminating $\Delta x$ the **normal equations** $A D^2 A^\top \Delta y = r$. As $\mu \to 0$, $d_j^2 \to \infty$ on the optimal-basis side and $\to 0$ off it; under degeneracy (non-unique partitions — SCUC again) $\kappa(AD^2A^\top)$ grows like $\Theta(1/\mu^2)$ and Cholesky dies exactly near convergence — the repo's historical `natIPM` failure signature (NumericalError on degenerate NETLIB/SCUC).

**The cure is regularization + augmented form.** Add primal/dual proximal terms:

$$\begin{pmatrix} -(D^{-2} + \rho I) & A^\top \\ A & \delta I\end{pmatrix},\qquad \rho, \delta \sim 10^{-8} \ldots 10^{-6}.$$

This matrix is **quasidefinite** (Vanderbei): for *any* symmetric permutation it admits a stable LDLᵀ with diagonal $D$ of fixed signature — so a sparsity-driven ordering (AMD/METIS) can be chosen freely, which is exactly the CHOLMOD-simplicial-LDLᵀ path the recent commits take. The induced perturbation is removed by **iterative refinement** on the unregularized system (a few solves), and $\rho, \delta$ escalate only on factorization failure. Mehrotra predictor-corrector + Gondzio multiple centrality correctors on top; Ruiz equilibration before everything.

#### 2.4.2 Crossover

An interior optimum $x^\mu$ must become a *basic* optimum to warm-start the tree. Theory: Megiddo's strongly complementary partition $(\mathcal B^*, \mathcal N^*)$ is identified in the limit by ratios $x_j / s_j$; crossover (a) rounds tiny $x_j$/$s_j$ to their partition sides, (b) runs a **push** phase moving super-basic variables to bounds along null-space directions (primal push for primal-feasibility, dual push symmetric), (c) finishes with warm dual simplex cleanup. Cost is usually a small fraction of IPM time *if* the partition is clean; under degeneracy the partition is ambiguous and crossover degrades — mitigations: stop IPM earlier (µ ≈ 1e-8·scale, don't polish), let simplex cleanup do the last mile, and always bound crossover pivots by a budget with fallback to cold dual simplex on the original LP.

**Portfolio corollary.** Because IPM (no warm start, superb asymptotics), dual simplex (warm start, degeneracy-sensitive), and PDLP (matrix-free, GPU-scalable; see `lp_first_order_methods_review.md`) have complementary failure modes, the root should be a **concurrent race** with first-success-wins and cross-seeding (IPM/PDLP solution → crossover → basis for simplex). All three engines already exist in-repo; the race harness is pure orchestration (§4-P3).

### 2.5 Cutting planes: derivations, quality, and floating-point validity

The repo's separator suite includes GMI, transformed-tableau cMIR, lifted knapsack/mixed covers, implied bounds, mod-k, path-mixing, CGLP, and conflict cuts. Breadth is not evidence of correctness or speed: each family requires an explicit validity contract and independent frontier audit before it can participate in production solves.

#### 2.5.1 The simple MIR inequality (base lemma, with proof)

> **Lemma (simple MIR).** Let $X = \{(y, s) \in \mathbb Z \times \mathbb R_+ : y - s \le \beta\}$ and $f = \beta - \lfloor \beta \rfloor \in (0,1)$. Then
> $$y - \frac{s}{1-f} \;\le\; \lfloor \beta \rfloor \quad \text{is valid for } X.$$
> *Proof.* If $y \le \lfloor\beta\rfloor$: since $s \ge 0$, $y - s/(1-f) \le y \le \lfloor\beta\rfloor$. If $y \ge \lfloor\beta\rfloor + 1$: let $k = y - \lfloor\beta\rfloor \ge 1$ (integer). From $y - s \le \beta$, $s \ge y - \beta = k - f$. For integer $k \ge 1$, $k - f \ge k(1-f) \iff kf \ge f \iff k \ge 1$. Hence $s/(1-f) \ge k = y - \lfloor\beta\rfloor$. ∎

Everything below is this lemma applied to cleverly constructed single rows. The **MIR function** $F_f(a) = \lfloor a\rfloor + \frac{(f_a - f)_+}{1-f}$ (where $f_a = a - \lfloor a \rfloor$) is superadditive and nondecreasing with $F_f(\beta)=\lfloor\beta\rfloor$; for a mixed row $\sum_{j \in I} a_j x_j + \sum_{j \in C} g_j z_j \le \beta$ ($x \in \mathbb Z_+^I$, $z \ge 0$) the general MIR cut is

$$\sum_{j\in I} F_f(a_j)\, x_j \;+\; \frac{1}{1-f}\sum_{j \in C:\, g_j < 0} g_j z_j \;\le\; \lfloor \beta \rfloor,$$

i.e. positive continuous coefficients are *dropped* (relaxation) and negative continuous mass is scaled by $\frac{1}{1-f}$.

#### 2.5.2 GMI from a tableau row = MIR + best complementation

Take an optimal-tableau row for a basic integer variable with fractional value: after shifting every nonbasic to 0 (complement at-upper variables $x_j \to u_j - x_j$),

$$x_{B_i} + \sum_{j \in \mathcal N} \bar a_j x_j = \bar b, \qquad f_0 = \bar b - \lfloor \bar b\rfloor \in (0,1),\ f_j = \bar a_j - \lfloor \bar a_j\rfloor .$$

Applying MIR, choosing *per integer variable* the better of writing $\bar a_j = \lfloor\bar a_j\rfloor + f_j$ or $\bar a_j = \lceil \bar a_j\rceil - (1-f_j)$ (whichever survives MIR with the smaller coefficient), yields the **Gomory mixed-integer cut**

$$\sum_{\substack{j \in \mathcal N_I \\ f_j \le f_0}} f_j\, x_j \;+\; \sum_{\substack{j \in \mathcal N_I \\ f_j > f_0}} \frac{f_0 (1-f_j)}{1-f_0}\, x_j \;+\; \sum_{\substack{j \in \mathcal N_C \\ \bar a_j > 0}} \bar a_j\, x_j \;+\; \sum_{\substack{j \in \mathcal N_C \\ \bar a_j < 0}} \frac{f_0}{1-f_0}\,(-\bar a_j)\, x_j \;\ge\; f_0 .$$

Validity requires only: $x_{B_i}$ integer-constrained, correct bound status of every nonbasic (the complementation!), and an *exact* row. Each of these is a numerical trust boundary (§2.5.6). GMI cuts are **rank-1 split cuts**; empirically the split closure alone closes 71–98% of the integrality gap on MIPLIB (Balas–Saxena; Dash–Günlük–Lodi) — which is the theoretical license to concentrate on MIR/GMI-family quality rather than exotic families.

#### 2.5.3 cMIR: aggregation + complementation + scaling (Marchand–Wolsey)

The practical strength of MIR comes from *which row* you apply it to. cMIR searches a 3-dimensional heuristic space: **(a) aggregation** — combine ≤ k structural rows with weights to isolate a useful mixed-knapsack (bounded slack) structure; **(b) complementation** — for each bounded variable choose $x_j$ or $u_j - x_j$ (typically: complement if LP value is closer to $u_j$); **(c) scaling** — divide the row by a candidate $\delta \in \{|a_j|\}$ of the integer coefficients (try a handful), then MIR; keep the most violated result. The repo's `xtab_*` pipeline (transform → cMIR → untransform → postprocess/tighten) mirrors the HiGHS implementation of exactly this scheme, including the variable-bound substitution ($x_j \le u_j y_j$-type) that turns dispatch-vs-commitment SCUC rows into strong mixed-knapsacks. Path-mixing cuts extend this along time-coupled (ramp/min-up) row chains — the domain-specific asset.

#### 2.5.4 Lifted cover cuts (knapsack rows)

For a knapsack $\sum_j a_j x_j \le b$, $x \in \{0,1\}^n$, a **cover** $C$ ($\sum_{j\in C} a_j > b$) gives $\sum_{j \in C} x_j \le |C| - 1$. Strengthening to a facet needs **lifting**: for $j \notin C$, the exact lifting coefficient is $\gamma_j = (|C|-1) - \max\{\sum_{i \in C} x_i : \sum_{i\in C} a_i x_i \le b - a_j\}$ — a knapsack itself, so codes use **sequence-independent lifting** via the superadditive lifting function $\phi$ of Gu–Nemhauser–Savelsbergh (valid for any lifting order, computable in $O(|C|)$ from the sorted cover weights). The repo's `xtab_separate_lifted_knapsack_cover` implements the GNS scheme (with `HighsCDouble` compensated arithmetic — the right call: lifting functions are cancellation-heavy).

#### 2.5.5 Mod-k / zero-half cuts

Chvátal–Gomory with multipliers restricted to $\{0, \tfrac1k, \ldots, \tfrac{k-1}{k}\}$: any $u \ge 0$ with $u^\top A$ integral except residues mod $k$ gives $\lfloor u^\top A\rfloor x \le \lfloor u^\top b \rfloor$ when $x \ge 0$ integer. For $k=2$: find row subsets whose coefficient parities cancel — separation reduces to GF(2) linear algebra on (rows mod 2 | rhs mod 2), exact separation of maximally violated zero-half cuts is NP-hard but Gaussian-elimination heuristics on the support of fractional variables are cheap and effective (Koster et al.). The repo's `add_transformed_modk_cuts` follows this line; the validity trust boundary is the **integrality of the aggregated row** — coefficients must be *verified* integral (snap-and-check with tolerance, else reject), since a 1e-9 residue times a big multiplier silently invalidates the rounding.

#### 2.5.6 Numerical validity of cuts — the central B&C numerics problem

A tableau-based cut is computed from $\bar a = \rho^\top A$ where $\rho$ itself came from BTRAN with backward error $O(\varepsilon \kappa(A_B))$. The generated inequality can therefore be **invalid for the true problem** — it may cut off integer-feasible points, the deadliest failure mode in MILP (wrong answers, not slow answers). Defense in depth:

1. **Source hygiene:** refuse tableau cuts when $\kappa$-estimate or FTRAN residual checks exceed thresholds (`basis_tableau_cuts_admissible` exists — keep it wired to the κ monitor of §2.2.5); require $f_0 \in [\tau_0, 1-\tau_0]$ with $\tau_0 \approx 0.005$–$0.01$ (near-integral $f_0$ ⇒ division by $1-f_0$ amplifies error and the cut is weak anyway).
2. **Coefficient hygiene:** drop tiny coefficients *safely* — a coefficient $|\alpha_j| < \epsilon_c$ on a variable with bounds $[\ell_j, u_j]$ may be removed only by relaxing the rhs by $\max(\alpha_j \ell_j, \alpha_j u_j) - \min(\cdot)$-style worst case (move mass to the rhs, never just delete); cap **dynamism** $\max|\alpha| / \min|\alpha| \le 10^4$ (tree) / $10^6$ (root); cap support (density ≤ a few % of $n$ or an absolute limit); snap near-integer coefficients and re-verify violation after snapping.
3. **Margin:** add the cut as $\alpha^\top x \le \beta + \eta$ with $\eta = \varepsilon_{\text{feas}} \cdot \max(1, \lVert\alpha\rVert_\infty \cdot \text{bound scale})$, and only if the LP-point violation exceeds $10\times$ that margin. This converts "possibly invalid by $O(\text{err})$" into "valid unless the error exceeds the margin", at negligible strength cost.
4. **Safe rounding (gold standard, root-level option):** compute the cut with **directed rounding** (Cook–Dash–Fukasawa–Goycoolea): for a $\ge$-cut over $x \ge 0$, round every coefficient *down* and the rhs *up* (interval-arithmetic FTRAN for the tableau row; per-sign directed rounding for general bounds). The result is provably valid in exact arithmetic. Costs ~2–4× per cut; reserve for root GMI rounds and for "paranoid mode" (§5).
5. **Reference-solution assertion (debug harness):** carry a known-feasible solution (e.g. from a HiGHS run or a previous incumbent); assert **no accepted cut ever violates it** beyond tolerance and no node containing it is ever pruned by bound. This single harness catches invalid cuts, wrong cutoffs, and bad propagation at their birth site instead of as end-to-end wrong answers (§5, D1). 
6. **Pool discipline:** cuts age out when non-tight for $k$ consecutive LPs (slack basic); pool capped; duplicates/dominated filtered by hash + parallelism test. LP conditioning degrades with accumulated near-parallel cut rows — monitor κ before/after cut rounds (§2.2.5) and purge aggressively. *Selection* per round: greedy by efficacy $\frac{\alpha^\top x^* - \beta}{\lVert \alpha \rVert_2}$ (or directed cutoff distance toward the incumbent, HiGHS-style), skipping candidates with pairwise parallelism $\frac{|\alpha_i^\top \alpha_j|}{\lVert\alpha_i\rVert\lVert\alpha_j\rVert} > 0.9$–$0.99$ against already-selected ones. This is precisely the fix for "MIR correct but hurts": a family should never be disabled wholesale; its candidates should lose the selection tournament when weak.

### 2.6 Branching theory

**Objective.** Choose the fractional variable whose dichotomy most shrinks the *tree*, not the one that looks most fractional. With per-branch dual-bound gains $g_j^- , g_j^+$, the standard score is the **product rule** $\mathrm{score}(j) = \max(g_j^-, \epsilon)\cdot\max(g_j^+, \epsilon)$ (Achterberg; $\epsilon \approx 10^{-6}$), preferred over weighted sums because closing the gap requires *both* children to progress — in the Le Bodic–Nemhauser abstract model, tree size for gap $G$ scales like $2^{G/g}$ per side, making the balanced-gain variable exponentially better than an unbalanced one with the same sum.

**Estimation.** Exact gains require solving both children (**full strong branching** — node-optimal, prohibitively expensive). The practical estimator stack:
- **Pseudocosts:** running averages of observed gain per unit fractionality from actual branchings, $\Psi_j^\pm$; prediction $\hat g_j^- = \Psi_j^- f_j$, $\hat g_j^+ = \Psi_j^+ (1-f_j)$.
- **Reliability branching (Achterberg–Koch–Martin):** until a variable has $\eta_{\text{rel}} \approx 4$–$8$ pseudocost observations per direction, run *budgeted strong branching* on the top candidates (warm dual simplex, iteration cap ~500–1k, bound-only probes); afterwards trust pseudocosts. Degrades gracefully to SB at the root (where it matters most) and to pure pseudocost deep in the tree.
- **Signal enrichment under degeneracy:** on SCUC-class models, many SB probes return $\Delta = 0$ (dual degeneracy ⇒ the LP bound doesn't move even for productive branchings). The tie-breaker chain must then use *non-LP* signals: inference counts (propagation tightenings triggered), conflict scores (VSIDS-style, exponentially decayed), cutoff counts. The repo's `PseudoCost` struct already tracks all three plus variance/LCB — the missing piece is only the reliability orchestration and the degeneracy-aware fallback ordering (§4-P6).

**Domain-specific branching (SCUC edge):** branch on *aggregated* commitment decisions (e.g. unit on/off over a time window, or $\sum_t u_{g,t}$ thresholds) rather than single binaries — a form of branching on general disjunctions that matches min-up/down structure; pairs naturally with the path-mixing cuts.

### 2.7 Presolve and node propagation

**Activity-based bound tightening (the workhorse).** For row $\sum_j a_{ij} x_j \le b_i$ define minimal activity $L_i = \sum_{a_{ij}>0} a_{ij}\ell_j + \sum_{a_{ij}<0} a_{ij} u_j$ (max activity $U_i$ symmetric, with explicit infinity *counters*, never $\pm10^{20}$ arithmetic — playbook N7). Then for any $j$ with $a_{ij} > 0$:

$$x_j \;\le\; \ell_j + \frac{b_i - L_i}{a_{ij}}\,, \qquad \text{tighten } u_j \text{ if this improves it by} \ge \varepsilon_{\text{step}}.$$

Integer $x_j$: round the new bound inward. Iterate rows↔columns to a fixpoint (bounded rounds). **Numerics contract:** computed with outward-safe margins ($L_i$ under-, never over-estimated: subtract $\varepsilon_a \cdot \text{scale}$), and a *minimum tightening step* $\varepsilon_{\text{step}} \ge 10\,\varepsilon_{\text{feas}}$ so tolerance-noise never masquerades as a reduction (else presolve "tightens" feasible points away one ulp at a time).

**Coefficient tightening (integer rows).** For $\le$-row with binary $x_j$, $a_{ij} > 0$: if $a_{ij} > b_i - U_{i,-j}$ where $U_{i,-j}$ is the max activity of the rest, then replace $a_{ij} \leftarrow b_i - U_{i,-j}$ and $b_i \leftarrow b_i$ adjusted accordingly — the constraint's integer solutions are unchanged but its LP relaxation strictly tightens. This is MIR-flavored preprocessing and is disproportionately effective on big-M/indicator rows (SCUC startup/ramp constraints): it is *the* systematic big-M reducer (playbook N8).

**Probing.** Tentatively set binary $x_j = 0/1$; propagate to fixpoint; harvest: fixings (both branches infeasible ⇒ infeasible; one ⇒ fix), implications $x_j = v \Rightarrow x_k \in [\cdot]$ (→ implication graph → clique table via merging), and lifted bounds. Quadratic-ish cost ⇒ budgeted (candidate ordering by fractionality/occurrence, work limits). The implication graph then feeds clique cuts, clique merging in presolve, and propagation strength at nodes.

**Further canonical reductions** (each with a dual-side counterpart): singleton rows/cols, doubleton equality substitution, parallel row/col detection (hashing), dominated columns, dual fixing (all-favorable-sign columns), redundant row deletion, implied-integrality detection. PaPILO provides most of this externally (with its own postsolve stack); the *native* additions that matter for this codebase are probing + clique table + coefficient tightening at root, and **node-level propagation** (already present: `propagate_node_domain` with implication graph + conflict pool) kept at fixpoint discipline with strict budgets.

**Postsolve exactness.** Every reduction must store an exact inverse transformation (including objective offset and dual/basis mapping). The round-2.5 bug class (objective reported in reduced space; fake gaps) generalizes to a rule: **all cross-boundary reporting (bounds, incumbents, gaps) happens in original space only**, converted at one choke point (§5, N11).

### 2.8 Conflict analysis

When a node dies, learn *why* — turning one pruned node into a constraint that prunes cousins.

**Propagation conflicts:** the node's infeasibility has a derivation DAG of bound changes; cutting it at the first unique implication point (1-UIP, SAT-style) yields a minimal-ish set of branching bounds $\{x_{j} \le/\ge v_j\}$ whose conjunction is infeasible ⇒ learn the **bound-disjunction clause** $\bigvee_j (x_j \gtrless v_j^{\text{neg}})$. The repo already learns clause-form conflicts from propagation failures and converts binary ones to cuts/implications.

**LP infeasibility conflicts (Farkas):** an infeasible node LP with local bounds $\ell', u'$ yields a Farkas certificate $y \ge 0$ with the aggregated row $\alpha^\top x \ (\alpha = y^\top A)$ satisfying $\min_{\ell' \le x \le u'} \alpha^\top x > y^\top b$. Evaluate that minimum term-by-term: each term uses $\ell'_j$ (if $\alpha_j > 0$) or $u'_j$ (if $\alpha_j < 0$). The *local* bounds actually used, restricted to those with $\alpha_j \ne 0$, form the conflict set; relax any subset back toward global bounds while the strict inequality (with a safety margin $\ge \varepsilon_{\text{feas}} \lVert y \rVert_1$-scaled) survives, to shorten the clause. **The certificate is free**: it is the dual ray the dual simplex already has when it detects unboundedness. This is the main missing conflict source in the repo (propagation conflicts exist; Farkas conflicts don't) — cheap, high-value (§4-P6). Bound-exceeding nodes (dual bound ≥ cutoff) yield the analogous *dual-proof* constraint from the optimal dual solution.

**Numerics caveat:** conflict constraints derived from a certificate computed at tolerance are only as valid as the margin used in the relaxation step — same margin discipline as cuts (§2.5.6, item 3); a corrupted Farkas ray (e.g. from an escalation-level solve that failed the residual audit) must never be harvested (§5, N3/N2 interplay).

---

## 3. Gap analysis: this codebase vs HiGHS

### 3.1 What HiGHS actually is (the target's anatomy)

As of the vendored HiGHS 1.14: a world-class **dual simplex** (`HEkk`: DSE pricing, BFRT, Harris ratio test, cost perturbation + shifting, PAMI/SIP parallel minor iterations) on top of **HFactor** (Markowitz LU, Forrest–Tomlin updates, native rank repair, hyper-sparse FTRAN/BTRAN — the Huangfu–Hall lineage); a solid MIP layer (presolve, `HighsDomain` propagation with cliques and implied bounds, conflict-ish analysis, transformed-LP separators: cMIR/tableau/clique/zero-half-family, symmetry detection with orbital fixing/orbitopes, RENS/RINS/rounding/shifting + feasibility-jump heuristics, pseudocost branching); IPX interior point with crossover; cuPDLP-C compiled in. Its relative weaknesses (the openings): tree parallelism is limited compared to commercial codes; presolve is shallower than Gurobi's (probing depth, nonlinear-ish reductions); no safe/exact numerics mode (demonstrably false-optimal under adversarial scaling — measured here); heuristics narrower than SCIP's; no learning/portfolio layer; no domain-structure exploitation.

### 3.2 Measured position of the native stack (from the 2026-07-20 evaluation, rounds 1–4)

| Layer | State vs HiGHS | Evidence |
|---|---|---|
| B&C orchestration (cuts, seeding, fallbacks, heuristics) | **Ahead** on 118-bus when running on HiGHS LP kernel | gap 5.9% vs 19.1% @120 s |
| Cut suite breadth | Comparable-to-ahead (incl. domain path-mixing, CGLP) | code inventory §0 |
| Propagation/conflict/heuristics/branching infra | Present, competitive design; needs Farkas conflicts + reliability orchestration | §2.6–2.8 |
| Presolve | PaPILO external (good); native probing/clique/coeff-tightening thin; `PresolveManager` stub | first-order review §1.1 |
| **Dual simplex kernel speed** | **~4× slower (39-bus), ~100–1000× slower per LP at 118-bus scale** | eval §3.4, §5.7 |
| **Warm-start robustness** | **8/10 tree warm starts die (UMFPACK-singular drifted bases)** | eval §5.7–5.8 |
| Phase I at scale | Fails on presolved 118-bus root (structural density, not scaling) | eval §6.1 |
| Numerical honesty | **Ahead** (residual audits; rejects false optima HiGHS accepts) | eval §3.2 |
| IPM root | Recovering (CHOLMOD augmented-KKT LDLᵀ, 13/15 scaled; health probe gates) | commits e1ea432…93122f9 |
| PDLP third pillar | Prototype + unexploited cuPDLP-C in-tree | first-order review |
| L2O / config learning | Framework exists (`l2o/`), not yet wired to B&C decisions | `docs/learning_to_optimize_framework.md` |

**Strategic reading.** The tree-level IP is already built and partially superior; the *foundation* (simplex kernel) is the bottleneck, and its failure modes are precisely the three classical capabilities described in §2.1–2.3 that UMFPACK-plus-eta cannot provide: FT-class updates with stability sentinels, in-factorization rank repair, and hyper-sparse solves. This is why the improvement must be *fundamental* (kernel architecture) rather than incremental (tolerance tuning around UMFPACK) — the eval doc's round-3/4 record is an empirical proof that tuning around the wrong factorization architecture dead-ends.

---

## 4. Engineering plan

Phases are ordered by leverage; each has concrete deliverables, owner files, and **measurable exit criteria** on the §1.2 protocol. Rough sizing assumes one focused engineer + this assistant; parallelize P4/P5/P6 freely once P1–P2 land.

### P0 — Measurement discipline and guardrails (week 0–2)

The eval infrastructure is already strong (`native_kernel_comparison --check`, residual audits, `[LP-STATS]`). Additions:
1. **SGM/profile harness** over (SCUC family × seeds × limits) + MIPLIB-2017 subset (start with the ~50 easiest; grow), emitting one canonical JSON consumed by a trend dashboard; HiGHS raw-API baseline rows always included. Files: extend `benchmark/native_kernel_comparison.cpp`, new `benchmark/miplib_runner.cpp`.
2. **LP iteration-parity probe:** for a fixed instance set, log (pivots, time/pivot, %hyper-sparse results, refactor count, κ estimates) native-vs-HiGHS side by side. Without this, P1 cannot be steered. Files: `src/engine/kernel/lp_kernel/*` stats plumbing.
3. **κ(A_B) estimator** (Hager–Higham via existing FTRAN/BTRAN) surfaced in `[LP-STATS]` and thresholded into cut-admissibility (§2.5.6.1).
4. **Reference-solution tracer** (§5, D1) as a first-class debug mode: `BCOptions::debug_reference_solution` + assertions at cut-accept/prune/propagate sites.
   *Exit: dashboards running nightly on dev; every later phase must show its SGM delta here.*

### P1 — Simplex kernel program (months 0–3; the make-or-break)

Target the §2.2 architecture directly inside `src/engine/kernel/lp_kernel/native_dual/` + `linear_algebra/`:
1. **Factorization swap:** finish the HFactor port as the default basis engine (fresh-build + native eta already works; fix or bypass `updateFT` — the broken row-eta content is localized per eval round 4). Requirements: build-time rank repair with logical substitution (§2.2.3) wired to warm starts; FT update with small-pivot refactor trigger. A/B gate vs `UmfpackNativeA` on the P0 parity probe.
2. **Hyper-sparse FTRAN/BTRAN/PRICE** (§2.2.4): Gilbert–Peierls symbolic DFS + indexed-nonzero vectors + density-based representation switching; CSR mirror of $A$ for row ops. Measure %hyper on SCUC relaxations (expect ≫50%).
3. **DSE** with Forrest–Goldfarb updates + weight-negativity alarms (§2.1.3); partial/Devex fallback when DSE's extra FTRAN dominates.
4. **Ratio test:** Harris two-pass + BFRT with stability-preferring tie-breaks (§2.1.2) — replacing absolute pivot floors.
5. **Anti-degeneracy:** magnitude-aware cost perturbation + removal contract, EXPAND working tolerance (§2.3).
6. **Adaptive refactorization** by the cost-balance rule + error triggers (§2.2.5).
   *Exit: ≤2× HiGHS pivot counts and ≤5× time/pivot on 39-bus/24T relaxation; presolved 118-bus root solves cold; ≥1k node-LPs/300 s at 118-bus (from 10). Then re-run the eval doc's §4 table.*

### P2 — Warm-start robustness (month 2–4, overlaps P1)

1. Basis repair on singular refactorization (from P1.1) replaces the fail→crash-repair→cold chain; DSE weights reset only for repaired rows.
2. Node LP protocol: bound-changes-only via direct $x_B$ update (no refactor), cut-append via slack-basic extension, parent-basis + parent-DSE-weight inheritance, fail-fast caps retained as backstop.
3. Farkas-certificate audit before any conflict harvesting (§2.8).
   *Exit: PATH-A warm ok-rate ≥95% (from 1/10); `fail_ms` <5% of LP time; zero false infeasibility certificates on the probe suites (HiGHS cross-check stays green).*

### P3 — Root portfolio (month 3–5)

1. Concurrent root race: dual simplex ∥ regularized IPM (+crossover, §2.4.2) ∥ PDLP (and optionally the in-tree cuPDLP-C as a zero-cost baseline), first-usable-basis-wins with cross-seeding; kill losers by budget. Extends the existing escalation/probe machinery in `bc/root/root_solve.cpp`.
2. IPM: keep hardening the augmented-KKT LDLᵀ path (regularization escalation $\rho,\delta$ ↑, iterative refinement, early-stop µ + simplex last mile).
   *Exit: 118-bus root (presolved or not) < 10 s natively; no configuration routes through a known-fragile kernel without a probe.*

### P4 — Presolve deepening (month 4–6, parallelizable)

Native layer complementing PaPILO: probing with budgets → implication graph/clique table at root (feeding existing clique machinery), coefficient tightening (§2.7 — the big-M reducer for SCUC), implied integrality, symmetry detection (identical generators! orbital fixing on unit-permutation orbits), plus the §2.7 numerics contract (outward rounding, min-step, infinity counters). Node-level: keep propagation fixpoint budgets; add objective propagation to cutoff-tighten bounds (`bc_objective_propagation` exists — verify wiring).
   *Exit: node counts on SCUC family ↓ ≥30% at equal correctness; no presolve-attributable audit failures.*

### P5 — Cut system consolidation (month 4–6, parallelizable)

The families exist; build the *management* layer of §2.5.6: unified pool + tournament selection (efficacy × orthogonality, per-round caps, aging), κ-aware admissibility, margins/safe-drop everywhere, re-enable MIR/cMIR under the tournament (never wholesale-off), root cut-loop scheduling by measured bound-improvement-per-second tail-off.
   *Exit: root gap closed on SCUC family ≥ HiGHS at equal time; zero reference-solution violations across the suite (D1 harness).*

### P6 — Branching, conflicts, heuristic scheduling (month 5–7)

Reliability branching orchestration over the existing `PseudoCost` infra (SB budgets on warm LPs, $\eta_{\text{rel}}$, product rule, degeneracy-aware tie-break chain: inference → conflict → cutoff scores); Farkas + dual-proof conflict harvesting (§2.8); heuristic budget manager (cost-based scheduling of the existing RENS/RINS/FP/diving stack, success-adaptive).
   *Exit: SCUC tree sizes competitive with HiGHS at equal node-LP speed; measurable incumbent-earliness improvement (primal integral).*

### P7 — Parallelism and the asymmetric edges (month 6+)

1. **Tree parallelism** (infrastructure exists in `bc/parallel/`): deterministic-by-default (node-count-based synchronization), work-stealing pool, concurrent separators/heuristics at the root.
2. **SCUC structure**: path-mixing/min-up-down cut families extended; aggregated-disjunction branching (§2.6); Benders integration for multi-period/contingency structure (module exists).
3. **L2O**: wire `l2o/` model fingerprints to (cut family weights, branching parameter presets, root-portfolio choice, heuristic budgets) — offline-trained on the benchmark farm. This is a genuine capability HiGHS does not have and is where "beat" can become "dominate" on the target domain.
4. **Safe-numerics mode as a feature**: rational re-check of final incumbents + Neumaier–Shcherbina safe dual bounds (cheap interval evaluation of $y^\top b + \sum \min(\cdot)$) for certified results — sellable in power-market settings where HiGHS's silent false optima (measured, §3.2) are disqualifying.

---

## 5. The numerical playbook for branch-and-cut

Branch-and-cut is an error *amplifier*: an LP solved to 1e-9 feeds a tableau cut whose coefficients feed a propagation pass whose bounds feed the next LP — across ~10⁵–10⁶ nodes. The defense is architectural: **one tolerance hierarchy with explicit dominance invariants, audits at every trust boundary, and a catalog of known failure modes with detectors.** Several entries below were *discovered the hard way* in this repo (eval rounds 1–4) and are canonized here.

### 5.1 Tolerance architecture

All tolerances are **relative to problem scale** (row norms / bound magnitudes) unless stated; raw absolute comparisons against sentinel values ($\pm 10^{20}$) are forbidden in arithmetic (see N7).

| Symbol | Meaning | Default | Dominance invariant |
|---|---|---|---|
| $\varepsilon_{\text{mach}}$ | double precision | 2.2e-16 | — |
| $\varepsilon_{\text{pivot}}$ | LU threshold-pivot $\tau$ | 0.1 (escalate → 1.0) | — |
| $\varepsilon_{d}$ | dual feasibility / Harris relaxation | 1e-7 | $\ge 10^2\,\varepsilon_{\text{mach}}\kappa$ |
| $\varepsilon_{\text{LP}}$ | LP primal feasibility | 1e-7 | — |
| $\varepsilon_{\text{feas}}$ | MIP row/bound feasibility | 1e-6 | $\ge 10\,\varepsilon_{\text{LP}}$ |
| $\varepsilon_{\text{int}}$ | integrality tolerance | 1e-6 | $\ge \varepsilon_{\text{LP}}$ |
| $\varepsilon_{\text{cut}}$ | min cut violation to accept | 1e-5 | $\ge 10\,\varepsilon_{\text{feas}}$ |
| $\eta_{\text{cut}}$ | cut rhs safety margin | $\varepsilon_{\text{feas}}\cdot\text{scale}$ | $\le \varepsilon_{\text{cut}}/10$ |
| $\varepsilon_{\text{step}}$ | min presolve/propagation tightening | 1e-5·scale | $\ge 10\,\varepsilon_{\text{feas}}$ |
| $\varepsilon_{\text{gap}}$ | cutoff slack: prune iff bound ≥ incumbent − $\varepsilon_{\text{gap}}\max(1,|inc|)$ | 1e-9 | $\ll \varepsilon_{\text{feas}}\cdot|c|$-scale |
| audit tol | independent original-space audit | 10× solver claim | strictly looser than claims |

The invariant column is the point: **every consumer of a quantity must tolerate more error than its producer guarantees.** A cut is accepted only when violated by ≥ $\varepsilon_{\text{cut}}$, which exceeds its own validity margin $\eta_{\text{cut}}$, which exceeds the LP error $\varepsilon_{\text{LP}}$ that produced the tableau — so a numerically marginal cut can be *weak* but not *wrong*. Violations of dominance are how tolerance bugs are born; a CI assertion should check the chain at option-parse time (options can be user-overridden into inconsistency).

Special rule — **integral objectives:** if all objective coefficients of integer variables are integral and continuous variables have zero cost (test after presolve, with gcd scaling), the cutoff may be strengthened to $\text{incumbent} - 1 + \varepsilon_{\text{gap}}$; the integrality test must be exact-rational on the *original* data, never tolerance-based on presolved data.

### 5.2 Failure-mode catalog

Format: **symptom → mechanism → detection → mitigation** (repo status in brackets).

- **N1. Invalid cut cuts off the optimum.** Wrong final answer, often "Optimal" with worse objective than a reference run; or child infeasibility storms. Mechanism: tableau error at high κ, aggregation with bad multipliers, unverified mod-k integrality, unsafe coefficient dropping. Detection: D1 reference tracer; per-cut re-substitution audit against the *original* rows. Mitigation: full §2.5.6 stack. [Partially present: `basis_tableau_cuts_admissible`, unscaling checks; margins + κ-gating + D1 to add.]
- **N2. False-optimal LP.** Kernel reports Optimal, solution violates original-space rows at 1e-1 — the HiGHS-at-10⁶-scaling failure. Detection: independent residual audit in original space (never in scaled space). Mitigation: audit gate + escalation; treat "Optimal" from any kernel as a *claim*. [**Done** — this repo's audit is the reference implementation; keep it mandatory for every kernel including HiGHS rows.]
- **N3. Warm basis singular after bound changes / cut rows.** Node LPs burn seconds in doomed refactorizations then cold-start. Mechanism: degenerate pivot admission on drifted bases (§2.2.3). Detection: `umf-numeric` refactor codes (instrumented), DSE weight alarms. Mitigation: in-factorization rank repair + ratio-test stability preference + perturbation; fail-fast caps as backstop. [Diagnosed; fix = P1/P2.]
- **N4. Cycling/stalling on degenerate vertices.** No objective progress over $O(m)$ pivots. Detection: progress watchdog (pivots since last strict dual-objective increase). Mitigation: §2.3 triple (perturb + EXPAND + BFRT); as last resort, random restart of pricing weights. [BFRT present; perturbation ported but default-off; EXPAND absent.]
- **N5. Perturbation leakage.** Perturbed problem's solution returned as if exact, or perturbation makes tight equalities infeasible. Rule: perturb only costs and finite bounds, never equality RHS; always run removal + cleanup re-solve; report only cleaned solutions. [Learned in round 1 — RHS perturbation produced false infeasibility certificates on power-balance rows.]
- **N6. Cutoff prunes the true optimum.** Mechanism: bound/cutoff compared across objective spaces (presolve offset!), or $\varepsilon_{\text{gap}}$ vs $\varepsilon_{\text{int}}$ stack-up, or integral-objective rounding applied when a continuous cost exists. Detection: D1 tracer (assert reference node never pruned); final gap audit best_bound ≤ audited incumbent. Mitigation: single-choke-point objective-space conversion; §5.1 invariants. [Offset bug found & fixed round 2.5; keep the regression test.]
- **N7. Sentinel poisoning.** $\pm10^{20}$ "no bound" values entering arithmetic (activity sums, audit scales, objective) → thresholds inflate to meaninglessness. Rule: infinities are *flags with counters*, excluded from all norms/scales; any $|value| \ge 10^{19}$ in a numeric path is a bug. Detection: debug-mode NaN/huge-value traps at API boundaries. [Bitten twice (audit scale, best_bound sentinel); scrubbers in place — generalize to activity computations in P4.]
- **N8. Big-M pathologies (SCUC native).** M ≫ row scale ⇒ feasibility tolerance × M = real violations of the logic the M encodes; LP bound weak; branching signals flat. Mitigation: coefficient tightening (§2.7) as the systematic M-reducer; per-row *relative* feasibility checks; report violation/row-norm in audits; prefer indicator-style modeling upstream in `scuc/` generators where possible. [Coefficient tightening thin natively — P4.]
- **N9. IPM breakdown near convergence.** Cholesky failure / NumericalError as µ→0 on degenerate LPs (§2.4.1). Mitigation: regularization escalation, refinement, early-stop + crossover + simplex last mile; health probe before committing (exists). [Improving on recent commits; probe done.]
- **N10. Cut-pool-induced conditioning collapse.** κ(A_B) grows with accumulated near-parallel cuts; node LPs slow down and N3 frequency rises. Detection: κ trend vs pool size in `[LP-STATS]`. Mitigation: aging + parallelism filter + per-round caps (§2.5.6.6). [Pool exists; tournament + κ feedback = P5.]
- **N11. Postsolve/space-mapping mismatches.** Objectives, bounds, duals, or bases reported in the wrong space; "gaps" between quantities from different spaces. Rule: original space is the only reporting space; conversions at one choke point with round-trip unit tests (feed solution → presolve → postsolve → assert identity within $\varepsilon_{\text{mach}}$-scale). [Round-2.5 class; choke point partially done.]
- **N12. Parallel nondeterminism masking bugs.** Failures that vanish under debug. Mitigation: deterministic mode (fixed node-scheduling order, seeded RNG everywhere, logged seeds), single-thread repro extraction; only then optimize opportunistically. [Parallel search exists — verify a deterministic switch in P7.]
- **N13. Incumbent adoption without audit.** Heuristic/repair paths publishing solutions that fail original-space feasibility (the round-1 "garbage incumbent 2.1e11"). Rule: *every* incumbent, from every path (heuristics, repair, adopt-root), passes the same audit gate before adoption; quality-gate vs dual bound (exists). [Done at root/repair; extend to all heuristic adoption sites.]
- **N14. Round-then-infeasible integrality.** $x_j = 0.9999997$ accepted as integral, fixed, then the LP is infeasible or the "integral" solution violates rows at $\varepsilon_{\text{int}}\cdot\lVert a\rVert$. Rule: near-integrality is a *hypothesis* — verify by re-solving the LP with integers fixed (cheap, warm) before claiming a feasible MIP solution; polish with one lexicographic re-solve. [Verify current incumbent path in P6.]

### 5.3 Debugging protocol (in escalation order)

- **D1. Reference-solution tracer:** run with a known-good solution (HiGHS or earlier incumbent); assert at every cut acceptance, propagation tightening, and prune decision that the reference survives. Converts end-to-end wrong answers into first-failure stack traces. (Highest ROI item in this whole section.)
- **D2. Trust-boundary audits always-on:** LP claims, incumbent adoption, presolve round-trips — already the house style; keep them cheap enough to never disable.
- **D3. Deterministic minimal repro:** deterministic mode → bisect nodes → dump (SF-LP, basis, bound set) at first failure (proof-artifact machinery exists — `bc_proof_artifacts.cpp`); replay standalone through the kernel harness.
- **D4. FP-environment hardening in CI:** one nightly job with FE traps (invalid/overflow) + ASan/UBSan on the unit tier; NaN provenance beats NaN archaeology.
- **D5. Exact cross-check for small repros:** rational-arithmetic LP/cut re-verification (Boost.Multiprecision or SoPlex-exact style) on extracted repros — settles "invalid cut vs tolerance mirage" arguments definitively.

---

## 6. Win conditions and sequencing

**Tier 1 (≈3 months, P0–P3): beat HiGHS end-to-end natively on the SCUC family.** The orchestration already wins on 118-bus when borrowing HiGHS's kernel; Tier 1 = making the native kernel good enough that the whole stack wins without borrowing. Measurable: SGM(native) < SGM(HiGHS raw) on the SCUC set at 1 thread and n threads, all runs passing audits.

**Tier 2 (≈6–9 months, +P4–P6): ≤1.3× HiGHS SGM on a MIPLIB-2017 benchmark subset.** Generality costs more than domain wins; 1.3× SGM with strictly better robustness (audit-clean, honest statuses — already demonstrated) is a strong, publishable position.

**Tier 3 (9+ months, +P7): beat HiGHS on the general benchmark.** Honesty required: HiGHS embodies ~15 engineer-decades of concentrated simplex/MIP craft; matching it feature-for-feature on its home turf is a long game. The realistic route to *beating* it is asymmetric: (a) domain structure (SCUC cuts/branching/Benders), (b) the L2O layer HiGHS lacks, (c) parallel tree + root portfolio, (d) certified-numerics mode as a differentiator, (e) GPU-PDLP roots at scales where simplex/IPM saturate. Winning on the target domain first funds the general campaign with credibility and regression armor.

**Anti-goals** (documented so they stay dead): tolerance-tuning around UMFPACK instead of replacing the architecture (empirically dead-ended, rounds 3–4); enabling cut families wholesale without tournament selection (empirically harmful, MIR episode); benchmarking without correctness gates (produces HiGHS-style false wins the audit already catches); perturbing equality RHS for Phase I.

---

## 7. References

**Simplex & linear algebra:** Koberstein, *The dual simplex method — techniques for a fast and stable implementation* (PhD, Paderborn, 2005) — the single best implementation reference; Forrest & Goldfarb, *Steepest-edge simplex algorithms for LP* (Math. Prog. 57, 1992); Fourer, *Notes on the dual simplex method* (unpublished, 1994) — BFRT; Maros, *Computational Techniques of the Simplex Method* (Kluwer, 2003); Harris, *Pivot selection methods of the Devex LP code* (Math. Prog. 5, 1973); Forrest & Tomlin, *Updated triangular factors of the basis…* (Math. Prog. 2, 1972); Suhl & Suhl, *Computing sparse LU factorizations…* (ORSA JoC 2, 1990); Hall & McKinnon, *Hyper-sparsity in the revised simplex method…* (Math. Prog. 102, 2005); Huangfu & Hall, *Parallelizing the dual revised simplex method* (Math. Prog. Comp. 10, 2018) — the HiGHS core; Gill, Murray, Saunders & Wright, *A practical anti-cycling procedure…* (EXPAND; Math. Prog. 45, 1989); Gilbert & Peierls, *Sparse partial pivoting in time proportional to arithmetic operations* (SIAM JSSC 9, 1988).

**IPM:** Mehrotra (SIAM J. Opt. 2, 1992); Gondzio, *Multiple centrality corrections…* (COAP 6, 1996); Vanderbei, *Symmetric quasidefinite matrices* (SIAM J. Opt. 5, 1995); Altman & Gondzio, *Regularized symmetric indefinite systems in IPMs* (OMS 11/12, 1999); Bixby–Saltzman and Megiddo on basis identification/crossover (ORSA JoC, 1994; Math. Prog. 1991).

**Cuts:** Marchand & Wolsey, *Aggregation and MIR to solve MIPs* (Oper. Res. 49, 2001); Nemhauser & Wolsey (1988) ch. II.1; Gu, Nemhauser & Savelsbergh, *Lifted cover inequalities…* (INFORMS JoC 10, 1998; sequence-independent lifting: Math. Prog. 85, 1999); Caprara & Fischetti, *{0,½}-Chvátal–Gomory cuts* (Math. Prog. 74, 1996); Balas & Saxena, *Optimizing over the split closure* (Math. Prog. 113, 2008); Dash, Günlük & Lodi (Math. Prog. 121, 2010); Cook, Dash, Fukasawa & Goycoolea, *Numerically safe Gomory mixed-integer cuts* (INFORMS JoC 21, 2009); Fischetti, Lodi & Tramontani, *On the separation of disjunctive cuts* (Math. Prog. 128, 2011) — CGLP normalization.

**Branch-and-cut architecture:** Achterberg, *Constraint Integer Programming* (PhD, TU Berlin, 2007) — the SCIP bible: propagation, conflict analysis, reliability branching, cut selection; Achterberg, Koch & Martin, *Branching rules revisited* (ORL 33, 2005); Le Bodic & Nemhauser, *An abstract model for branching…* (Math. Prog. 166, 2017); Achterberg & Wunderling, *MIP: analyzing 12 years of progress* (Springer, 2013) — component-impact ablations; Berthold, *Primal heuristics for MIP* (PhD, TU Berlin, 2014); Fischetti & Lodi, *Local branching* (Math. Prog. 98, 2003); Fischetti, Glover & Lodi, *The feasibility pump* (Math. Prog. 104, 2005); Witzig, Berthold & Heinz, *Experiments with conflict analysis in MILP* (CPAIOR 2017).

**Numerics:** Neumaier & Shcherbina, *Safe bounds in LP and MIP* (Math. Prog. 99, 2004); Gleixner, Steffy & Wolter, *Iterative refinement for LP* (INFORMS JoC 28, 2016); Wilkinson backward-error framework (any edition); Curtis & Reid, *On the automatic scaling of matrices…* (JIMA 10, 1972); Ruiz, *A scaling algorithm…* (RAL-TR-2001-034); Klotz, *Identification, assessment, and correction of ill-conditioning and numerical instability in LP and MIP* (INFORMS TutORials, 2014) — the practitioner's numerics catalog.

**First-order LP:** see `docs/lp_first_order_methods_review.md` (PDLP/rHPDHG/HPR-LP litreview, this repo, 2026-07-28).

**In-repo companions:** `docs/native_vs_highs_milp_evaluation.md` (measured baseline & failure forensics), `docs/native_dual_simplex_rewrite.md`, `docs/native_dual_stabilization.md`, `docs/numerical_methods.md`, `docs/learning_to_optimize_framework.md`.

---

## 8. Addendum (2026-07-28): kernel code audit — the P1/P2 plan made concrete

A source-level audit of the *current* kernel (post-eval-doc rewrite: `dual_simplex.cpp` now routes into `src/engine/kernel/lp_kernel/native_dual/` on top of `HFactorBackend`) updates the §3.2 diagnosis. **The architecture gaps named in §2 are largely already closed** — the vendored HFactor is the basis engine with FT updates (`hfactor_backend.cpp:129`), rank repair with logical substitution exists (`factorize_with_logicals`, no-pivot export at `hfactor_backend.cpp:158-163`), DSE with Forrest–Goldfarb updates and Devex fallback are implemented (`pricing.cpp:432`), and the ratio test is a Harris two-pass BFRT with flip transactions and cost shifts (`pricing.cpp:122`). What remains is not architecture but **two cost multipliers stacked on the hot loop**, plus Phase-I strategy:

### 8.1 Bottleneck 1 — the per-pivot 100×: dense solves × always-on audits

Per dual iteration the kernel currently performs (files/lines as of `dev` today):

| Step | Cost today | Reference |
|---|---|---|
| CHUZR checked BTRAN | dense solve **+ full O(nnz(B)) residual audit** (+ refine) | `factor.cpp:391-489` |
| PRICE | dense $\rho^\top A$, O(nnz(A)) every pivot | `solver.cpp:102` |
| Ratio-test error bounds | `dot_error_bound` traverses each candidate's column ≈ another O(nnz(A)) pass | `pricing.cpp:24,144` |
| + two more O(n) scans (cost shifts, step interval) | `pricing.cpp:318,357` |
| Entering FTRAN | dense solve + full residual audit (+ refine) | `solver.cpp:143` |
| `pivot_evidence` | **two full long-double passes over all basis columns, every pivot** | `factor.cpp:125-203` |
| `row_solve_consistent` | two more full long-double basis passes | `factor.cpp:205-295` |
| DSE update | extra checked FTRAN + O(m); exact-BTRAN fallback on cancellation | `pricing.cpp:491-546` |
| Reduced-cost update | dense O(n) vector op | `solver.cpp:322` |

That is ~10–15 full passes over problem data per pivot (several in long double), where HiGHS does ~2–4 **hyper-sparse** passes over tiny reachable sets. Decisively: the `HFactorBackend` wrapper calls the **dense** `ftranCall(std::vector<double>&)` overload, and the `_for_update` HVector variants pass `expected_density = 1.0` (`hfactor_backend.cpp:345-421`) — so HiGHS's sparse/hyper-sparse solve strategies (§2.2.4), *which are compiled and sitting in the vendored factor*, are never engaged. The 100–1000× ≈ (dense-vs-hypersparse: 10–100×) × (audit overhead: ~3–5×).

**Plan (workstream A, replaces §4-P1 items 1–2 which are done):**
- **A1. Engage hyper-sparsity:** drive FTRAN/BTRAN through `HVector` with real sparse index sets (BTRAN rhs is a *unit vector* — count = 1, the canonical hyper-sparse case) and maintained expected-density running averages per solve type (as `HEkk` does), instead of dense buffers + density 1.0. Preserve the result's index/count through the wrapper (indexed vectors alongside `VectorXd`) instead of flattening.
- **A2. Tier the audit machinery** (do NOT delete it — it is the safety net that ended the UMFPACK forensics era): Tier 0 every pivot, O(1)–O(sparse): row/column pivot agreement (the one check HiGHS makes), NaN guards, DSE-weight positivity. Tier 1 every k pivots and at rebuild: one full residual audit. Tier 2 (today's full evidence suite: `pivot_evidence`, `row_solve_consistent`, per-candidate `dot_error_bound`): only on Tier-0/1 trips, in escalation levels, and under a `SimplexOptions::paranoid` flag (kept ON for root/final/published solves in B&C).
- **A3. Sparse PRICE:** CSR mirror of A; compute the pivot row only over the support of $\rho$ (row-wise traversal), O(support-row nnz) instead of O(nnz(A)); replace per-candidate `dot_error_bound` column traversals with a precomputed column-norm scalar bound.
- **A4. Sparse updates:** reduced-cost update over the pivot row's support only; nonbasic-candidate index list instead of three O(n) scans.

*Gate: the §4-P0 pivot-parity probe; target ≤5× HiGHS time/pivot on 39-bus, then 118-bus relaxations.*

### 8.2 Bottleneck 2 — warm-start deaths: likely stale diagnosis, one real tension

The 8/10 warm failures were diagnosed on the **old UMFPACK+eta architecture**; the new kernel factors warm bases through HFactor with rank repair at every rebuild and never cold-discards a warm basis (`native_dual_core.hpp` contract). Two actions:
- **B1. Re-measure first** (cheap): re-run the round-3 `--warm-probe` / tree-chain probes and the 118-bus B&C row on the `native_dual` path; collect `statistics.rank_repairs` and fail reasons. Only then tune.
- **B2. The real remaining tension:** HFactor's rank tolerance (`pivot_tol` default 1e-10) *declares mildly ill-conditioned SCUC bases rank-deficient where UMFPACK succeeded* (comment at `hfactor_backend.cpp:130-132`). Round 4's "whack-a-mole" was repair-churn on exactly such bases. Policy: repair-and-continue is correct for a *few* rows; add a churn cap (repairs > ~5% of m in one rebuild → treat as failed warm start, fall back to logical basis) plus taboo of the repaired rows' re-entry (taboo machinery exists). DSE weights of repaired rows reset to 1; all others persist.
- **B3. Cross-node reuse:** `Result` already returns `edge_weights` and basis — B&C should round-trip both (parent → child hint), eliminating DSE re-initialization ($m$ BTRANs in `compute_exact_edge_weights`) per node.

### 8.3 Bottleneck 3 — Phase I at scale: avoid it, then replace it

The cold controller runs an artificial-objective **primal** Phase I (`native_dual_core.hpp`). Strategy:
- **C1. Make Phase I rare:** tree nodes never need it (§2.1.1); a crossed-over IPM root basis is dual feasible; only the raw cold root remains. The §4-P3 root portfolio therefore mostly *removes* this bottleneck.
- **C2. For the residual cold case, switch to dual Phase I via cost shifting** on the all-logical basis (trivially factorizable): shift costs to dual feasibility, run the *same* warm Phase-II machinery, remove shifts through the existing mandatory original-cost cleanup pass. One code path for cold and warm; no artificial variables; no primal Phase I at 27k rows in dense+audited mode.
- **C3. Re-test the presolved-118-bus cliff after A1** — its "structural density" hardness (eval §6.1) was measured on dense solves; hyper-sparse solves change the calculus for denser-but-smaller matrices.

**Execution order:** A1 → A2 (independent, biggest product) → B1 re-measurement → A3/A4 → C2 → B2/B3 tuning — each step gated on the parity probe + `native_kernel_comparison --check` staying green.

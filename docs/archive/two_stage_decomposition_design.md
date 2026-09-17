# Two-Stage Decomposition Module — Design and Theoretical Derivation

Module location:

- `include/mipsolvers/engine/decomposition/two_stage.hpp`
- `src/engine/decomposition/two_stage.cpp`
- Tests: `tests/test_benders_decomposition.cpp`

The module lives in `mipsolvers::engine::decomposition` and depends only on the
public solver engine (`SolverEngine::solve_lp` / `solve_milp`). It has no
dependency on SCUC, power-system, or any domain data structure. It is the
in-tree, Gurobi-free counterpart of the MATLAB reference algorithms in
`DecompositionAlgorithms/` (`two_stage_so_centralized.m`,
`two_stage_so_with_mixed_integer_recourse_ccg.m`, …).

It provides three exact solvers for the two canonical two-stage problem classes:

| Problem class | Algorithm | Entry point |
|---|---|---|
| Two-stage **stochastic** program (risk-neutral expectation) | **L-shaped / Benders** decomposition (single- and multi-cut) | `solve_benders_stochastic` |
| Two-stage **robust** optimization (finite / discrete uncertainty) | **Column-and-Constraint Generation (CCG)** | `solve_ccg_robust` |
| Either, as a monolithic baseline | **Extensive form** (deterministic equivalent) | `solve_extensive_form_*` |

The extensive-form solvers are the correctness oracle: on every test instance the
decomposition optimum must match the monolithic optimum to `1e-6`.

---

## 1. Standard form

Let the **first stage** (here-and-now) be

$$
\min_{x}\; c^\top x \quad\text{s.t.}\quad A x \le b,\; A_{\mathrm{eq}} x = b_{\mathrm{eq}},\; x \in X,
$$

where $X$ encodes finite bounds and (optionally) integrality of $x\in\mathbb{R}^{n_1}$.

For each **scenario** $s = 1,\dots,S$ (a realization $\omega_s$ of the uncertainty)
the **second-stage / recourse** value function is

$$
Q_s(x)\;=\;\min_{y}\; d_s^\top y
\quad\text{s.t.}\quad W_s\, y \;\ge\; h_s - T_s\, x,\; y \in Y_s .
\tag{R$_s$}
$$

$T_s$ is the *technology matrix* coupling the two stages, $W_s$ the *recourse
matrix*, $h_s$ the recourse right-hand side, $Y_s$ the recourse variable domain
(bounds and, for robust CCG, optional integrality). This `≥` convention matches
the MATLAB reference (`W y >= h - T x`).

The two problem classes differ only in how the scenario values are aggregated:

$$
\textbf{Stochastic:}\quad \min_{x\in X}\; c^\top x + \sum_{s=1}^{S} p_s\, Q_s(x),
\qquad p_s>0,\ \textstyle\sum_s p_s = 1 .
\tag{SP}
$$

$$
\textbf{Robust:}\quad \min_{x\in X}\; c^\top x + \max_{s=1,\dots,S}\; Q_s(x).
\tag{RO}
$$

(RO) is the discrete/finite-uncertainty two-stage robust problem: the adversary
picks the worst realization from the finite set $\{\omega_1,\dots,\omega_S\}$,
which for polyhedral uncertainty are taken as its extreme points.

---

## 2. Structure of the recourse value function

**Assumption A1 (continuous recourse for Benders).** For (SP) the recourse
variables are continuous, $Y_s = \{y : \ell_s \le y \le u_s\}$.

**Lemma 1 (convex piecewise-linear recourse).** Under A1, $Q_s(\cdot)$ is convex
and piecewise-linear on its (polyhedral) domain
$\operatorname{dom} Q_s = \{x : (\mathrm{R}_s)\text{ is feasible}\}$.

*Proof.* LP strong duality applied to (R$_s$) gives, wherever the primal is
feasible and bounded,

$$
Q_s(x) \;=\; \max_{\pi \ge 0,\;\rho}\; \pi^\top (h_s - T_s x) + \rho^\top(\text{bounds})
\quad\text{s.t.}\quad W_s^\top \pi + (\text{bound terms}) \le d_s .
\tag{D$_s$}
$$

The dual feasible set $\Pi_s = \{\pi \ge 0 : W_s^\top\pi \le d_s\}$ does **not**
depend on $x$. Hence $Q_s$ is the pointwise maximum of the finitely many affine
functions $x \mapsto \pi^\top(h_s - T_s x)$ indexed by the vertices of $\Pi_s$,
which is convex and piecewise-linear. $\square$

**Corollary 1 (subgradient).** If $\pi_s^\star$ is an optimal dual of (R$_s$) at
$x_k$, then $-T_s^\top \pi_s^\star \in \partial Q_s(x_k)$ and

$$
Q_s(x)\;\ge\; Q_s(x_k)\;-\;\bigl(T_s^\top \pi_s^\star\bigr)^\top (x - x_k)
\qquad \forall x \in \operatorname{dom} Q_s .
\tag{OPT}
$$

This is the **Benders optimality cut**. The slope follows from the envelope
theorem: only the coupling right-hand side $h_s - T_s x$ depends on $x$, and
$\partial Q_s/\partial(\text{rhs}) = \pi_s^\star$, so
$\partial Q_s/\partial x = -T_s^\top \pi_s^\star$; bounds $\ell_s,u_s$ do not
depend on $x$ and therefore contribute nothing to the slope (the intercept is
pinned exactly by the known value $Q_s(x_k)$).

### 2.1 Sign convention in this codebase (verified)

The engine returns `constraint_duals` ordered `[inequality rows | equality rows]`
with the **HiGHS effective-minimization sign convention**: for a `≤` row that is
active at its upper side the dual is $\le 0$
(`src/engine/kernel/lp_kernel/dual_simplex.cpp`). We build (R$_s$) in the
engine's `≤` form by negation,

$$
W_s y \ge h_s - T_s x \;\Longleftrightarrow\; \underbrace{(-W_s)}_{A_{\mathrm{sub}}} y \;\le\; \underbrace{T_s x_k - h_s}_{b_{\mathrm{sub}}},
$$

and let $\mu_s = \texttt{constraint\_duals}$ be the returned multipliers of those
rows. Because the `≥`-multiplier $\pi_s = -\mu_s$, the subgradient in (OPT)
becomes

$$
-T_s^\top \pi_s^\star \;=\; T_s^\top \mu_s .
$$

So the implemented optimality cut for scenario $s$ is

$$
\boxed{\;\theta_s \;\ge\; Q_s(x_k) \;+\; \bigl(T_s^\top \mu_s\bigr)^\top (x - x_k)\;}
\tag{OPT'}
$$

which is exactly the generic-Benders formula previously validated in this
repository (subproblem $Ay \le b - Bx$, gradient
$g_j=-\sum_i B_{ij}\mu_i$, with $A=-W_s,\,B=-T_s$). The runtime guard
$\text{LB} \le \text{UB} + \varepsilon$ (Section 4) fails loudly if this sign is
ever wrong, and the extensive-form cross-check pins it in the test suite.

### 2.2 Feasibility cuts (elastic recourse)

If (R$_s$) is infeasible at $x_k$ (recourse cannot absorb $x_k$), (OPT) is
undefined and we instead cut off $x_k$. Rather than depend on backend-specific
Farkas rays, we solve the **elastic feasibility subproblem** (Birge & Louveaux,
*Introduction to Stochastic Programming*, 2nd ed., §5.1):

$$
w_s(x)\;=\;\min_{y \in Y_s,\; v \ge 0}\; \mathbf{1}^\top v
\quad\text{s.t.}\quad W_s y + I v \;\ge\; h_s - T_s x .
\tag{FEAS$_s$}
$$

(FEAS$_s$) is always feasible and bounded, and $w_s(x)=0 \iff x \in \operatorname{dom}Q_s$.
$w_s$ is convex piecewise-linear (Lemma 1 applied to (FEAS$_s$)); with optimal
coupling-row dual $\sigma_s$ (recovered as $\sigma_s = -\mu_s^{\text{feas}} \ge 0$),

$$
\boxed{\;0 \;\ge\; w_s(x_k) \;+\; \bigl(T_s^\top \mu_s^{\text{feas}}\bigr)^\top (x - x_k)\;}
\tag{FEAS'}
$$

is a valid inequality that removes $x_k$ (its left/right values at $x_k$ are
$0 \ge w_s(x_k) > 0$, a contradiction, so $x_k$ is excluded) while retaining every
$x$ with feasible recourse.

**Relatively complete recourse.** When every $x\in X$ has feasible recourse (as
in the power-system instances of `DecompositionAlgorithms/`, whose recourse
carries load-shed / curtailment slacks), (FEAS$_s$) never triggers and only
optimality cuts (OPT') are generated.

---

## 3. Algorithms

### 3.1 L-shaped / Benders for (SP)

**Master (multi-cut).** With one epigraph variable $\theta_s$ per scenario:

$$
\min_{x\in X,\,\theta}\; c^\top x + \sum_{s} p_s \theta_s
\quad\text{s.t.}\quad A x \le b,\ A_{\mathrm{eq}}x=b_{\mathrm{eq}},\ \theta_s \ge L_s,\ \{(\text{OPT'}),(\text{FEAS'})\}.
$$

**Master (single-cut).** One aggregated epigraph $\theta = \sum_s p_s\theta_s$ and
one aggregated optimality cut per iteration
$\theta \ge \sum_s p_s\bigl[Q_s(x_k) + (T_s^\top\mu_s)^\top(x-x_k)\bigr]$.
Multi-cut converges in fewer major iterations at the price of $S$ rows per
iteration; single-cut keeps the master small. Both are provided.

**Valid initial lower bound $L_s$.** To keep the first master bounded we compute,
once, the global under-estimator

$$
L_s \;=\; \min_{x,\,y}\; d_s^\top y \quad\text{s.t.}\quad W_s y + T_s x \ge h_s,\; A x \le b,\; A_{\mathrm{eq}} x = b_{\mathrm{eq}},\; x\in \widehat X,\; y \in Y_s,
$$

with $\widehat X$ the continuous relaxation of $X$. Since it minimizes over the
relaxed first stage, $L_s \le \min_{x\in X} Q_s(x) \le Q_s(x^\star)$, hence
$\theta_s \ge L_s$ is valid. If this LP is **proven** unbounded below, the module
requires a finite user-supplied `theta_lower_bound`; the default NaN rejects the
solve because no automatically verifiable lower bound is available. Numerical
failure, interruption, or an ambiguous solver status never activates the
fallback.

**Iteration.**
1. Solve the master $\Rightarrow (x_k,\theta_k)$; set $\text{LB} = $ master objective.
2. For each $s$: solve (R$_s$). If optimal, accumulate
   $\text{UB}_{\text{cand}} \mathrel{+}= p_s Q_s(x_k)$ and add (OPT'); else solve
   (FEAS$_s$) and add (FEAS').
3. If all scenarios feasible, $\text{UB} = \min\bigl(\text{UB},\, c^\top x_k + \sum_s p_s Q_s(x_k)\bigr)$.
4. Stop when $\text{UB}-\text{LB} \le \varepsilon\max(1,|\text{UB}|)$.

**Theorem 1 (finite convergence).** Under A1, Benders terminates at a global
optimum of (SP) in finitely many iterations. *Sketch.* By Lemma 1 the cut
families (OPT'),(FEAS') are generated from vertices/rays of the finitely many
polyhedra $\Pi_s$; only finitely many distinct cuts exist. LB is nondecreasing
(cuts only tighten the master) and every incumbent is a true upper bound, so no
$x_k$ repeats without closing the gap (Van Slyke & Wets 1969). $\square$

### 3.2 Column-and-Constraint Generation for (RO)

CCG (Zeng & Zhao, *Oper. Res. Lett.* 41(5), 2013) keeps a growing **active set**
$\mathcal{O} \subseteq \{1,\dots,S\}$ and, for each active scenario, adds its
recourse **variables (columns)** and **constraints** to the master:

$$
\min_{x\in X,\,\eta,\,\{y^s\}_{s\in\mathcal{O}}}\; c^\top x + \eta
\quad\text{s.t.}\quad A x \le b,\ A_{\mathrm{eq}}x=b_{\mathrm{eq}},\quad
\eta \ge d_s^\top y^s,\ \ W_s y^s \ge h_s - T_s x \ \ \forall s\in\mathcal{O}.
\tag{M$_\mathcal{O}$}
$$

**Oracle.** Given the master solution $x_k$, evaluate $Q_s(x_k)$ for all $s$ and
pick $s^\star = \arg\max_s Q_s(x_k)$; set $\text{UB} = c^\top x_k + \max_s Q_s(x_k)$
and $\text{LB} = $ (M$_\mathcal{O}$) objective. If some scenario is
recourse-infeasible at $x_k$, prefer it as $s^\star$ (adding its constraints
restores robust feasibility).

**Iteration.** Solve (M$_\mathcal{O}$); if $\text{UB}-\text{LB}\le\varepsilon$ stop;
else $\mathcal{O} \leftarrow \mathcal{O}\cup\{s^\star\}$ and repeat.

**Theorem 2 (exact finite convergence).** With $|U|=S<\infty$, CCG terminates at
a global optimum of (RO) in at most $S$ iterations. *Sketch.* (M$_\mathcal{O}$)
is a relaxation of (RO) restricted to $\mathcal{O}$, so its value is a valid LB;
$x_k$ evaluated by the oracle gives a valid UB. Each iteration adds a scenario
not previously binding, and there are only $S$ scenarios, so the active set
saturates in $\le S$ steps, at which point $\eta^\star = \max_s Q_s(x^\star)$ and
LB = UB (Zeng & Zhao 2013, Prop. 1). $\square$

**Mixed-integer recourse.** CCG needs no recourse duality: the oracle solves each
$Q_s(x_k)$ exactly (an LP or, if $Y_s$ has integrality, a MILP) and the master
embeds the exact recourse block. Thus (RO) is solved exactly even with
integer recourse — the case where classical Benders optimality cuts (OPT) fail.
This is why the robust path, not the stochastic path, carries the
integer-recourse capability.

### 3.3 Integer L-shaped for (SP) with integer recourse

For the **stochastic** program with integer recourse, the continuous cut (OPT)
is invalid because $Q_s$ is no longer convex. When the first stage is **pure
binary** ($X = \{0,1\}^{n_1}\cap\{A x\le b\}$), the **integer L-shaped** method
(Laporte & Louveaux, *Oper. Res. Lett.* 13, 1993) restores exactness with a
cut that is tight at the incumbent binary point and dominated elsewhere.

Let $L_s \le \min_{x\in X} Q_s(x)$ be a valid lower bound (Section 3.1's
under-estimator, with recourse integrality relaxed, still qualifies), and at a
binary $x_k$ define $S_1(x_k)=\{i: (x_k)_i=1\}$, $S_0(x_k)=\{i:(x_k)_i=0\}$.
The **integer optimality cut** is

$$
\boxed{\;\theta_s \;\ge\; \bigl(Q_s(x_k)-L_s\bigr)\Bigl(\textstyle\sum_{i\in S_1}x_i - \sum_{i\in S_0}x_i - (|S_1|-1)\Bigr) + L_s\;}
\tag{INT}
$$

**Proposition 3 (validity and tightness).** (INT) is valid for every binary
$x$ with $\theta_s\ge L_s$, and is tight at $x_k$.
*Proof.* At $x=x_k$ the bracket equals $|S_1|-0-(|S_1|-1)=1$, so the right side
is $Q_s(x_k)$ — tight. For any binary $x\neq x_k$, at least one bit differs, so
$\sum_{i\in S_1}x_i-\sum_{i\in S_0}x_i \le |S_1|-1$, making the bracket
$\le 0$; since $Q_s(x_k)-L_s\ge0$, the right side is $\le L_s\le Q_s(x)$, so the
cut is implied by the bound $\theta_s\ge L_s$ and removes no feasible point.
$\square$

$Q_s(x_k)$ is computed **exactly** by solving the recourse MILP at $x_k$; no
recourse dual is required. In addition, a supporting hyperplane of the recourse
*LP relaxation* $Q_s^{\mathrm{LP}}$ at $x_k$ satisfies
$\text{plane}(x)\le Q_s^{\mathrm{LP}}(x)\le Q_s(x)$, so the continuous cut (OPT')
built from LP-relaxation duals remains a **valid** lower-bounding cut and is
added alongside (INT) to accelerate the bound. Feasibility is handled by (FEAS')
on the LP relaxation under the relatively-complete-recourse assumption.

**Theorem 3 (finite convergence).** With pure-binary first stage and bounded
integer recourse, the integer L-shaped method terminates at a global optimum of
(SP) in finitely many iterations, since each binary $x_k$ is visited at most
once (its (INT) cut forces $\theta_s\ge Q_s(x_k)$ there) and $\{0,1\}^{n_1}$ is
finite (Laporte & Louveaux 1993). $\square$

### 3.4 CCG with polyhedral (continuous) uncertainty

The finite-scenario CCG of §3.2 extends to a **continuous polyhedral uncertainty
set** with right-hand-side uncertainty and continuous LP recourse:

$$
\min_{x\in X}\; c^\top x + \max_{u\in U}\, Q(x,u),\qquad
Q(x,u)=\min_{y\ge 0}\{d^\top y : W y \ge h_0 + P u - T x\},
$$

with $U=\{u : u^{\mathrm{lb}}\le u\le u^{\mathrm{ub}},\ G u\le g\}$ a bounded
polytope (box + budget rows). The master is identical to (M$_\mathcal{O}$): each
uncertainty realization $u^k$ produced by the oracle contributes a recourse
block with fixed right-hand side $h_0 + P u^k$, so the finite-CCG master and its
finite-convergence argument apply verbatim, with the extreme points of $U$
playing the role of the finite scenario set.

**Oracle (adversarial max–min).** The only new component evaluates, for a fixed
$x^\star$,

$$
\max_{u\in U}\; Q(x^\star,u)\;=\;\max_{u\in U}\ \min_{y\ge 0}\{d^\top y : W y \ge r(u)\},\quad r(u)=h_0+Pu-Tx^\star .
$$

Replacing the inner LP by its Karush–Kuhn–Tucker system turns this into a single
**mixed-integer program** (Zeng & Zhao 2013, §3): with dual $\pi\ge0$, primal
slack $s=Wy-r(u)\ge0$, and reduced cost $\rho=d-W^\top\pi\ge0$,

$$
\begin{aligned}
\max_{u,y,\pi,s,\rho,b,e}\quad & d^\top y \\
\text{s.t.}\quad & u\in U,\ W y - P u - s = h_0 - T x^\star,\ W^\top\pi+\rho=d,\\
& y\ge0,\ s\ge0,\ \pi\ge0,\ \rho\ge0,\\
& \pi_i \le M\,b_i,\ \ s_i \le M(1-b_i),\ b_i\in\{0,1\} && (\pi_i s_i = 0)\\
& y_j \le M\,e_j,\ \ \rho_j \le M(1-e_j),\ e_j\in\{0,1\} && (y_j\rho_j = 0).
\end{aligned}
\tag{ORACLE}
$$

The two complementarity families $\pi_i s_i = 0$ and $y_j\rho_j = 0$ enforce
inner optimality; the big-$M$ linearization is exact whenever $M$ bounds the
optimal $\pi,s,y,\rho$. **The implementation validates $M$ at runtime**: if any
$\pi,y,s,\rho$ attains $\ge 0.99M$ at the oracle optimum, the solve is rejected
with a diagnostic rather than returning a value from an active artificial bound
(design doc §4 mismatch protocol). Relatively complete recourse ($y\ge0$ with
slacks always feasible) is assumed so the inner LP is always feasible.

**Correctness.** (ORACLE) returns $\max_{u\in U}Q(x^\star,u)$ because its
feasible set is exactly $\{(u,y,\pi):u\in U,\ (y,\pi)\text{ solve the inner
KKT at }u\}$ and, at any such point, $d^\top y = Q(x^\star,u)$ by strong LP
duality; maximizing $d^\top y$ therefore maximizes $Q(x^\star,\cdot)$ over $U$.
CCG then adds the block for the returned $u^\star$ and repeats; finite
convergence follows from §3.2 applied to the (finitely many) active extreme
points of $U$.

### 3.5 Mixed-integer recourse in the adversarial oracle

The KKT oracle (ORACLE) is exact **only for continuous recourse**: it replaces
the inner $\min_y$ by its LP dual/KKT system, which requires LP strong duality.
With **integer recourse** the inner problem is a MILP; it has no dual with zero
gap, $Q(x,u)$ is no longer concave in $u$ (it is a lower envelope of finitely
many LP value functions, generally nonconcave and discontinuous), and the
worst-case $u$ need not be a vertex of $U$. Feeding integer recourse to (ORACLE)
would produce **invalid cuts**, so `solve_ccg_polyhedral_robust` rejects it
loudly. The exact options are, in increasing generality:

1. **Finite / discrete uncertainty — already exact.** When $U$ is finite (or
   replaced by its extreme scenarios), `solve_ccg_robust` needs no recourse
   duality: its oracle evaluates each $Q_s(x)$ by solving the recourse **MILP**
   directly, so integer recourse is handled exactly (§3.2). This is the
   recommended path whenever $U$ can be represented by finitely many scenarios.

2. **Nested column-and-constraint generation** (Zhao & Zeng 2012). Treat the
   oracle $\max_{u\in U}\min_{y\in Y}$ as a two-stage robust problem in its own
   right ($u$ is its "here-and-now", $y$ its recourse) and apply CCG to it: an
   inner master proposes $u$, an inner subproblem solves the recourse MILP at
   $u$ to return an integer recourse policy $\hat y^\ell$ (a new column), and the
   inner master maximizes over $u$ subject to $\eta_{\text{in}}\le d^\top\hat y^\ell$
   with the constraints of each retained policy. Because the recourse has
   finitely many optimal integer parts, the inner CCG terminates finitely and
   returns the exact worst case. This is the general exact method for continuous
   polyhedral $U$ with mixed-integer recourse.

3. **Lagrangian / SDDiP cuts** (Zou, Ahmed & Sun 2019). When the coupling
   (state) variables are **binary**, Lagrangian cuts obtained by dualizing a
   copy constraint $z=x$ are valid *and* tight and close the duality gap for
   integer recourse. These are the cuts prototyped in
   `DecompositionAlgorithms/..._BDD.m`; they also strengthen the integer
   L-shaped method of §3.3.

4. **Dual + McCormick reformulation** (`..._dual_relaxtion.m`). Split
   $y=(y_c,y_b)$; dualize the continuous block and linearize the resulting
   bilinear dual$\times$binary products with McCormick envelopes and big-$M$.
   Exact for the special structure with binary recourse; the source of the
   `big_m`/envelope constants must be documented, as here.

5. **$K$-adaptability** (Hanasusanto, Kuhn & Wiesemann 2015). Restrict the
   recourse to $K$ candidate policies chosen after $u$ is revealed, turning the
   problem into one MILP; exact as $K\to\infty$ and a strong heuristic for small
   $K$.

The module implements option 1 today (exact for MILP recourse under finite
uncertainty) and §3.3 for the stochastic integer-recourse case; option 2 (nested
CCG) is the designated extension for continuous polyhedral uncertainty with
mixed-integer recourse.

### 3.6 Lagrangian cuts (exact for a binary first stage)

For a **binary first stage** $x\in\{0,1\}^{n_1}$ and integer recourse, the
continuous cut (OPT) is invalid, but *Lagrangian cuts* (Zou, Ahmed & Sun 2019)
recover exactness. Introduce a per-scenario local copy $z_s=x$:

$$
Q_s(x)=\min_{y\in Y_s,\;z\in\{0,1\}^{n_1}}\{d_s^\top y : W_s y \ge h_s - T_s z,\ z=x\}.
$$

Relaxing $z=x$ with a multiplier $\lambda\in\mathbb{R}^{n_1}$ gives, for **any**
$\lambda$, an affine-in-$x$ under-estimator:

$$
\mathcal{L}_s(\lambda;x)=\lambda^\top x + v_s(\lambda) \le Q_s(x),\qquad
v_s(\lambda)=\min_{y\in Y_s,\,z\in\{0,1\}^{n_1}}\{d_s^\top y - \lambda^\top z : W_s y + T_s z \ge h_s\},
$$

so the **Lagrangian cut** is

$$
\boxed{\;\theta_s \;\ge\; \lambda^\top x + v_s(\lambda)\;}
\tag{LAG}
$$

The tightest cut at $x_k$ solves the Lagrangian dual $\max_\lambda \mathcal{L}_s(\lambda;x_k)$.
$v_s$ is concave and piecewise-linear in $\lambda$ with subgradient $-z_s^\star(\lambda)$,
so the dual is maximized by an inner cutting-plane loop over $\lambda\in[-B,B]^{n_1}$:
each step solves the inner MILP $v_s(\lambda)$ (a mixed-integer program in
$(y,z)$) and adds the supporting plane
$\varphi(\lambda)\le \varphi(\lambda^j)+(x_k-z_s^j)^\top(\lambda-\lambda^j)$,
$\varphi(\lambda)=\lambda^\top x_k+v_s(\lambda)$.

**Proposition 4 (exactness for binary state).** Because $z$ is binary, the
Lagrangian dual value equals $\min\{d_s^\top y\}$ over the closed convex hull of
the coupling set, so at a binary $x_k$ the best Lagrangian cut is tight,
$\theta_s=Q_s(x_k)$ (Zou, Ahmed & Sun 2019, Thm. 2). With finitely many binary
points the outer method (a binary-first-stage master accumulating (LAG) cuts)
terminates at a global optimum, exactly as in Theorem 3. Lagrangian cuts are
generally tighter than the integer optimality cut (INT), at the cost of an inner
dual solve per scenario and iteration.

The implementation solves the multiplier dual over the configured box
$[-B,B]^{n_1}$ by a cutting-plane method. It emits a cut as *tight* only when the
inner upper/lower gap is closed and the best evaluated multiplier that supplies
the cut is strictly inside the box. The next cutting-plane master trial can lie
on a tied boundary face and is not the lower-bound incumbent used in the cut.
Reaching `lagrangian_inner_iterations` or obtaining the best evaluated
multiplier on the box boundary fails with a diagnostic; accepting either case
would invalidate the finite-convergence claim above.

A tied cutting-plane face can first supply a boundary multiplier even when the
same optimum is attained in the interior. In that case the implementation
evaluates one contracted multiplier, $0.98\lambda$, with the exact inner MILP
and accepts it only when its value closes the existing master upper bound to the
same tolerance. This costs zero additional solves on the usual interior path
and exactly one additional inner MILP on the boundary-certificate path; failure
of that independent check still rejects the cut and requests a larger box.

The inner stopping test is
$U-L\le 10^{-7}(1+|L|)$. This is one order of magnitude tighter than the
default outer relative gap and above double-precision roundoff at the objective
scales admitted by the model contract. Thus the inner cut is certified to a
smaller numerical error than the outer convergence decision; a caller needing
a tighter outer gap must also use a backend whose solve tolerance supports it.


---

## 4. Cost model and quantitative predictions

Let $\nu$ = master iterations, $S$ = scenarios, $m_s,n_s$ = recourse rows/cols.

| Quantity | Multi-cut Benders | CCG |
|---|---|---|
| Master rows added / iter | $S$ (cuts) | $m_{s^\star}+1$ (constraints) |
| Master cols added / iter | $0$ | $n_{s^\star}$ (recourse block) |
| Subproblem solves / iter | $S$ LPs (parallelizable, warm-startable) | $S$ oracle solves |
| Dominant per-iter cost | 1 master MILP + $S$ LP | 1 (growing) master MILP + $S$ LP/MILP |
| Iteration bound | finite (Thm 1) | $\le S$ (Thm 2) |

**Prediction (registered before measurement).**
- *Correctness*: `|obj_Benders − obj_extensive| ≤ 1e-6` and
  `|obj_CCG − obj_robust_extensive| ≤ 1e-6` on all test instances.
- *Effort*: on the small structured test instances, Benders closes the gap in
  $\nu \le 10$ iterations and CCG in $\le 3$ iterations (both $\ll S$), because
  the recourse polyhedra expose few distinct active dual vertices.

**Mismatch protocol.** If a decomposition optimum disagrees with the extensive
form, or LB exceeds UB by more than $\varepsilon$ at any iteration, stop: the
first suspect is the dual sign/scaling in (OPT')/(FEAS') (implementation
infidelity), then the coupling-matrix orientation (assumption), then the theory.
Re-derive here before editing code, per the repository theory-guided contract.

---

## 5. Model contract (enforced, fails loudly)

- First stage and every recourse are **minimization** problems.
- Dimensions: $T_s \in \mathbb{R}^{m_s\times n_1}$, $W_s\in\mathbb{R}^{m_s\times n_s}$,
  $h_s\in\mathbb{R}^{m_s}$, $d_s\in\mathbb{R}^{n_s}$; all coefficients and
  right-hand sides and stored variable bounds are finite, and bounds are
  ordered. The project-wide large finite sentinels represent absent bounds.
- Stochastic probabilities satisfy $p_s>0$ and $|\sum_s p_s-1|\le
  10^{-10}S$. Robust solvers ignore the scenario probability field.
- `solve_benders_stochastic` with `MultiCut`/`SingleCut` requires **continuous
  recourse** (A1); an integer recourse variable is rejected with an explicit
  status, not silently relaxed.
- `solve_benders_stochastic` with `IntegerLShaped` accepts integer recourse but
  requires a **pure-binary first stage** (Proposition 3); a non-binary first
  stage is rejected with an explicit status.
- `solve_ccg_robust` accepts continuous **or** integer recourse.
- The polyhedral KKT oracle accepts exactly continuous recourse with lower bound
  zero and no finite upper bound. General variable bounds require additional KKT
  multipliers and complementarity pairs and are rejected.
- A backend `success` flag is not an optimality certificate: decomposition
  masters, cut subproblems, exact recourse evaluations, and KKT oracles accept
  only explicit optimal termination. Infeasible, unbounded, limited, ambiguous,
  and failed solves remain distinct and cannot update LB, UB, or a cut.
- Adapter statuses are matched by an explicit proof-bearing allowlist. In
  particular, CPLEX gap-tolerance optimality, SCIP feasible-limit solutions,
  HiGHS feasible-limit solutions, and native B&C gap termination are rejected.
  Even for an allowlisted status, the reported relative MIP gap must be at most
  `1e-8`, two orders of magnitude below the default outer gap `1e-6`; this keeps
  inner-solve uncertainty below the outer stopping decision.
- The elastic feasibility LP certifies infeasibility only when its optimum is
  greater than `1e-8`; a zero elastic value after an infeasible primal status is
  treated as a contradictory backend result.
- No unsupported path returns a fabricated number: unsupported or failed solves
  set `success=false` with a diagnostic `status`, and `objective` remains NaN.

---

## References

1. R. Van Slyke, R. Wets. *L-shaped linear programs with applications to optimal
   control and stochastic programming.* SIAM J. Appl. Math. 17(4), 1969.
2. J. Benders. *Partitioning procedures for solving mixed-variables programming
   problems.* Numer. Math. 4, 1962.
3. J. Birge, F. Louveaux. *Introduction to Stochastic Programming*, 2nd ed.,
   Springer, 2011 (§5.1 optimality/feasibility cuts; multi-cut §5.4).
4. B. Zeng, L. Zhao. *Solving two-stage robust optimization problems using a
   column-and-constraint generation method.* Oper. Res. Lett. 41(5), 2013.
5. G. Laporte, F. Louveaux. *The integer L-shaped method for stochastic integer
   programs with complete recourse.* Oper. Res. Lett. 13(3), 1993.
6. L. Zhao, B. Zeng. *An exact algorithm for two-stage robust optimization with
   mixed integer recourse problems.* Optimization Online, 2012.
7. J. Zou, S. Ahmed, X. A. Sun. *Stochastic dual dynamic integer programming.*
   Math. Program. 175, 2019 (Lagrangian cuts for binary state variables).
8. G. A. Hanasusanto, D. Kuhn, W. Wiesemann. *K-adaptability in two-stage robust
   binary programming.* Oper. Res. 63(4), 2015.
9. Reference MATLAB prototypes: `DecompositionAlgorithms/two_stage_so_*.m`.

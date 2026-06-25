# Theoretical Framework — Simulation-Based Optimal Planning of Distribution Systems

> Status: **Theory v0.2** (revised after adversarial review) · Owner: Matrixeigs · Date: 2026-06-24
> Companion to `simulation_based_planning_design.md` (roadmap), `…_interfaces.md` (contracts), `…_amortized_l2o.md` (acceleration). Changes vs v0.1 are summarized in `…_review_response.md`.
>
> **Positioning (deliberately narrowed).** This framework is a **bound-certified, simulation-verified, ML-accelerated** planning method. It does **not** claim full high-fidelity *global* optimality, nor unconditional finite termination, nor guaranteed cross-scale L2O generalization. Every guarantee below is explicitly conditioned on a stated model class, scenario set, and certificate.

---

## 0. The three modules are three mathematical objects

| Module | Repo / target | Formal object | Produces |
|---|---|---|---|
| **MIPSolvers** | `mipsolvers::mipsolvers` | optimization operator `𝒪` (solves algebraic MILP/LP + returns duals/bounds) | **valid lower bounds** (only) |
| **Simulation** | `hacdcpf` | evaluation oracle `𝒮` (materializes a feasible operation *trajectory* and its cost) | **valid upper bounds** (only) |
| **Planning** | new | model-management loop `𝓛` | the certified gap, refinements, ML proposals |

The framework is the precise definition of `𝒪`, `𝒮`, `𝓛` and an **anytime bound-certification** statement, with finite termination as a *conditional* corollary (§6).

---

## 1. The planning problem — and the crucial two-level recourse

### 1.1 Variables and spaces

- **First stage (investment)** `x ∈ 𝒳`. Siting/sizing of PV, ESS, VSC/ER, SOP, lines, shared/mobile ESS, microgrids. **For the certified results below we require `𝒳` to be a bounded integer lattice** (every capacity a discrete unit multiple `C_k = n_k Δ_k`, `n_k ∈ {0,…,N_k}`; every build a binary). Continuous `𝒳` is allowed but downgrades finite termination to *anytime* (§6).
- **Uncertainty** `ω ∈ Ω`, finite sample `Ξ` (§5).
- **Second stage (operation)** `y(ω) ∈ 𝒴(x,ω)` — the operation variables of an `hacdcpf` model.

### 1.2 Two recourse values — never conflate them (revision R-A)

This separation is the backbone of the corrected framework.

| Symbol | Meaning | Computable? | Role |
|---|---|---|---|
| `Q_H^opt(x,ω)` | the **ideal** optimal operation cost under the *true* high-fidelity physics (nonconvex AC/DC, multi-period, MINLP) | **No** (in general) | conceptual reference only |
| `Q̂_H(x,ω)` | cost of a **concrete, materialized, physically-feasible operation trajectory** that the simulator `𝒮` actually produces (when it reports Feasible) | Yes, by `𝒮` | **UB ingredient** |
| `Q_L(x,ω)` | optimal value of a **tractable relaxation** model solved by `𝒪` | Yes, by `𝒪` | **LB ingredient** (only if certified, §3) |

By construction (a real achievable operation is never cheaper than the ideal optimum, and a certified relaxation never exceeds the ideal optimum):

$$
\boxed{\;Q_L(x,\omega)\;\overset{\text{(cert. §3)}}{\le}\;Q_H^{\mathrm{opt}}(x,\omega)\;\overset{\text{(achievable)}}{\le}\;\widehat Q_H(x,\omega).\;}
\tag{$\star$}
$$

**We never assert `Q̂_H = Q_H^opt`** unless `𝒮` is proven to solve the recourse to certified global optimality (it is not). All bounds rest on the *inequalities* ($\star$), not on equality.

### 1.3 The target problem (ideal, over a fixed sample)

$$
(\mathrm{P_H}|\Xi_{\mathrm{opt}})\qquad
v^\*_\Xi \;=\;\min_{x\in\mathcal{X}}\; c^\top x + \sum_{s\in\Xi_{\mathrm{opt}}} p_s\, Q_H^{\mathrm{opt}}(x,\omega_s)
\quad\text{s.t.}\quad x\ \text{feasible }\forall\,\omega\in\Xi_{\mathrm{verify}},\ \ \text{carbon cap}.
$$

`v^*_Ξ` is what the loop **certifies a gap around** — not the true-distribution optimum (that is an SAA matter, §5.3). Scenario roles (`Ξ_opt`, `Ξ_verify`, …) are defined in §5.1.

---

## 2. The simulation oracle `𝒮` — three states, trajectory-grounded UB

`𝒮` returns one of **three** statuses (revision R-E); only the first two are usable for bounds.

$$
\mathcal{S}:(x,\omega,\text{fidelity})\mapsto
\begin{cases}
(\textsf{Feasible},\ \widehat Q_H,\ m) & \text{a concrete trajectory was materialized and satisfies all physical constraints}\\
(\textsf{Infeasible},\ +\infty,\ \text{cert}) & \text{provably no feasible operation; cert has } \texttt{CertificateValidity} \\
(\textsf{Indeterminate},\ \bot,\ \bot) & \text{solver failure / nonconvergence / timeout — no verdict}
\end{cases}
$$

**UB validity (revision R-F — joint trajectory).** A `Feasible` verdict counts toward the UB **only if** its `Q̂_H` is the cost of a *single materialized operation trajectory* per scenario that simultaneously satisfies every physical constraint, with all reported metrics (carbon, losses) **derived from that same trajectory**. Statistical/post-processed estimates (e.g. Monte-Carlo `EENS`, separately-solved restoration) are **not** standalone UB ingredients — they inform `Ξ_verify` feasibility and are reported as estimates, never folded into the deterministic cost UB. The `Full` fidelity level (`…_interfaces.md` §3) is defined precisely by this trajectory-consistency requirement.

**Indeterminate (revision R-E).** Treated as *neither* feasible *nor* infeasible: no UB update, no validity-critical cut. The loop escalates fidelity / retries / runs diagnostics; if unresolved the case is flagged **uncertified**. Finite-termination claims explicitly exclude Indeterminate returns.

Properties as before: expensive, gradient-free in `x`, possibly discontinuous, **reproducible** under fixed seed (tolerance-valid, §0 of interfaces).

---

## 3. The relaxation `Q_L` and its certificate (revision R-B)

`𝒪` needs an algebraic model. We use a low-fidelity recourse `Q_L` (LinDistFlow / SOC-DistFlow / DC + linear couplings). **The left inequality of ($\star$) is *not automatic*** — linearized power flow can silently *tighten* a true constraint, which would break the lower bound.

### 3.1 Relaxation Certificate Registry — LB validity is conditional

LB validity holds **only over the model class certified by a registry** of per-template lemmas:

| Constraint template | High-fidelity counterpart | Claimed relation | Assumptions | If violated |
|---|---|---|---|---|
| (SOC) DistFlow branch | AC branch flow | **outer (convex) relaxation** | radial/weakly-meshed | rigorous LB; preferred |
| LinDistFlow voltage drop | AC branch flow | *approximation* (not certified relaxation) | radial, balanced, small angle, low loss | **LB not certified** → see 3.2 |
| thermal ampacity | AC current limit | relaxation iff stated as `‖S‖≤S̄` outer poly | voltage lower bound | may tighten |
| converter capability | PQ capability circle | polyhedral **outer** approximation | known rating | valid iff outer |
| ESS SoC dynamics | same linear dynamics | **exact** | fixed-efficiency model | exact |
| load shedding @ VOLL | same / relaxed | relaxation | VOLL finite ⇒ complete recourse | — |

**Rule.** A constraint may enter `Q_L` only tagged `exact`, `outer-relaxation` (with a registry lemma), or `dropped`. Any constraint that cannot be certified as non-excluding of `𝒴_H` points must be **dropped** (which preserves the relaxation), never approximated-in-place.

### 3.2 Two grades of lower bound (honesty)

- **Certified LB (`LB^cert`)** — every active recourse constraint carries a registry lemma ⇒ `Q_L ≤ Q_H^opt` holds ⇒ `LB^cert ≤ v^*_Ξ`. Achievable with the **SOC-DistFlow** convex relaxation. **This is the only bound we call "valid for `(P_H)`."**
- **Model LB (`LB^model`)** — if LinDistFlow (an approximation) is used for speed, the master's optimum bounds only the *relaxation-model* problem, **not** `v^*_Ξ`. We report it as a model-internal estimate; the AC fidelity gap is then carried entirely by the oracle's `Q̂_H − Q_L` term, and the *certified* statement degrades to "UB is valid; LB is model-internal."

The MVP (design doc) must pick one: **SOC-DistFlow for a rigorous `LB^cert`**, or **LinDistFlow with the bound explicitly labeled `LB^model`.** No silent middle.

---

## 4. The corrected two-master architecture (revision R-LB — the central fix)

The v0.1 algorithm computed the lower bound from a point that minimized the *corrected* surrogate. That is invalid: for a minimization, the relaxation objective evaluated at any feasible point is an **upper** bound on the relaxation optimum, not a lower bound. The fix:

### 4.1 Master-L (the *only* source of LB)

$$
(M_L^k):\quad \min_{x\in\mathcal{X}}\; c^\top x + \sum_{s\in\Xi_{\mathrm{opt}}} p_s\,\theta_s
\quad\text{s.t.}\quad \theta_s \ge (\text{valid relaxation/optimality cuts}),\ \ \theta_s \ge \underline\theta_s,\ \ x\ \text{feasibility cuts}.
$$

Solved by `𝒪`. **`LB_k = ` optimal objective (equivalently dual bound) of `M_L^k`.** As certified cuts accumulate, `LB_k` is **monotone non-decreasing** and `LB_k ≤ v^*_Ξ`. *Nothing else may write `LB`.*

### 4.2 Master-C (proposal only — never a bound)

$$
(M_C^k):\quad \min_{x\in\mathcal{X}}\; c^\top x + \sum_{s} p_s\big(\theta_s + \hat e_s(x)\big)
$$

with the embedded residual surrogate `ê` (and/or warm-started by an L2O policy). Its solution `x̂_C^k` is a **candidate** sent to `𝒮` (for UB) and used to warm-start `M_L`. **Its objective value is discarded for bounding.** Degenerate-but-safe option: skip `M_C` entirely and obtain candidates by a rounding/local-search heuristic — the LB machinery is unaffected.

> Master-L and Master-C may be one model with a switchable objective; the invariant is simply: **LB is read only after solving with the pure objective and only-valid cuts.**

### 4.3 Cut validity gate

A cut may enter `M_L` only if **validated**: optimality/feasibility cuts from LP-recourse duals (tolerance-valid), integer L-shaped cuts for binary-`x` MILP recourse, or `OracleFeasibilityCut` from a `Validated` certificate (§2). `Heuristic`/`None` certificates and surrogate-derived inequalities may enter **only `M_C`**, never `M_L`.

---

## 5. Scenarios and the three distinct gaps (revisions R-D, R-G)

### 5.1 Scenario-set taxonomy — different roles, different math

| Set | Role | Enters | Weight |
|---|---|---|---|
| `Ξ_opt` | estimate the expectation | objective `Σ p_s Q` | probability `p_s` |
| `Ξ_verify` | must-pass feasibility (critical N-1/k, design resilience events) | **constraints** + feasibility cuts | none (hard) |
| `Ξ_audit` | out-of-sample generalization | **reports only** | none |
| `Ξ_stress` | adversarial stress tests | reports; optional robust/CVaR variant | scenario-dependent |

**Never** merge `Ξ_verify`/`Ξ_audit`/`Ξ_stress` into `Σ p_s Q`. Adversarially-discovered failures go to `Ξ_verify` (as feasibility, not as re-weighted expectation) unless the *objective is explicitly redefined* as robust/CVaR — in which case state it.

### 5.2 Three gaps — keep them separate

1. **Certified SAA optimization gap** `Gap_SAA = UB_Ξ − LB^{cert}_Ξ ≥ 0`, both on the **same** `Ξ_opt`. By ($\star$) this bounds the suboptimality of incumbent `x̂` for `(P_H|Ξ_opt)`; it contains the genuine optimization slack **plus** the fidelity slack `Q̂_H − Q_L` (conservative, correct).
2. **Sampling / generalization gap** `F(x;ℙ) − F(x;Ξ)` — pure SAA statistics, reported as a confidence interval, **not** a deterministic bound.
3. **Fidelity gap** `Q̂_H − Q_L` — measured directly by the oracle; the quantity the residual surrogate (`ê`) learns.

We **report `UB_Ξ` and `LB^cert_Ξ` on a common `Ξ_opt`**, and *separately* report the out-of-sample audit `F̂_audit(x) ± z_α σ̂/√N`. The deterministic gap and the statistical CI are never added together.

### 5.3 SAA caveat

`LB^cert_Ξ`, `UB_Ξ` certify `(P_H|Ξ_opt)`, an SAA of the true-distribution problem. Consistency and confidence come from independent batches (standard SAA theory). The loop is *not* claimed to bound the true-distribution optimum deterministically.

---

## 6. The master algorithm `𝓛` and its *conditional* guarantees

```
Input: PlanningCase; tolerance ε; Ξ_opt⁰, Ξ_verify⁰
LB ← −∞ ; UB ← +∞ ; x* ← ∅
repeat
  (1) LB STEP  — solve Master-L (M_L) with current valid cuts  (module 𝒪)
        LB ← optimal/dual bound of M_L            ▷ monotone ↑ ; the ONLY LB writer (§4.1)
  (2) PROPOSE  — solve Master-C / heuristic / L2O warm-start → candidate x̂   (§4.2)
  (3) VERIFY   — for ω in Ξ_opt ∪ Ξ_verify: 𝒮(x̂, ω, escalating fidelity)     (module 𝒮)
        case Feasible(Q̂_H, m):    accumulate trajectory cost            ▷ R-F joint-trajectory
        case Infeasible(cert):     if cert.Validated → add OracleFeasibilityCut to M_L (§4.3)
        case Indeterminate:        no UB, no validity-cut; escalate/retry; else mark uncertified
        if x̂ Feasible over all required ω:
            UB ← min(UB, cᵀx̂ + Σ_{Ξ_opt} p_s Q̂_H,s) ; if improved x* ← x̂   ▷ R-A achievable UB
  (4) REFINE   — relaxation/optimality cuts → M_L ; retrain ê (proposal only) ; grow Ξ_verify
until  UB − LB ≤ ε   (Gap_SAA on common Ξ_opt)   and no new Validated violation
report x*, Gap_SAA, and SEPARATELY the Ξ_audit statistics            ▷ §5.2 never merged
```

### 6.1 Assumptions

- (A1) `𝒳` bounded; for **finite** termination, `𝒳` is a finite integer lattice (§1.1).
- (A2) Complete recourse of `Q_L` (VOLL load-shed) ⇒ `Q_L < ∞`.
- (A3) `Ξ_verify` finite and enumerable.
- (A4) **Relaxation registry (§3.1) certifies the active model** ⇒ `LB = LB^cert` valid; else `LB = LB^model` (not a bound on `v^*_Ξ`).
- (A5) Oracle returns Feasible or Validated-Infeasible on every required `(x,ω)` (no unresolved Indeterminate).

### 6.2 Guarantees — stated at their true strength

> **Anytime certification (unconditional given A2,A4-cert).** At every iteration `LB^cert ≤ v^*_Ξ ≤ UB` whenever `UB < ∞`; `LB` is monotone non-decreasing; the best `UB` is non-increasing. `Gap_SAA = UB − LB^cert` is a valid, computable certificate of suboptimality for `(P_H|Ξ_opt)`.

> **Finite termination (conditional).** *Additionally* under (A1-lattice)+(A3)+(A5): only finitely many distinct first-stage points and finitely many validated cuts/columns exist, so the loop reaches `Gap_SAA ≤ ε` in finitely many iterations. **Remove any one of {discretized `𝒳`, finite `Ξ_verify`, no Indeterminate, validated cuts} and only the anytime property survives** — no general finite guarantee for continuous sizing (a `C_ess = 3.1729` vs `3.1730` no-good cut cannot finitely enumerate).

> **ML-neutrality.** `ê` and any L2O policy appear only in step (2)/`M_C`; by §4 they never write `LB` or `UB`. Hence both guarantees hold **verbatim regardless of ML quality**; ML changes only the iteration count.

### 6.3 What is *not* claimed
Global optimality of the nonconvex `(P_H)` over continuous `x`; equality `Q̂_H = Q_H^opt`; a deterministic bound on the true-distribution optimum; finite termination with continuous sizing or Indeterminate returns. These are out of scope **by design**, and saying so is what makes the in-scope claims defensible.

---

## 7. Carbon–reliability coupling (unchanged in spirit, tightened in placement)

ε-constraint scalarization traces the `cost↔carbon↔reliability` Pareto frontier:
$$
\min_x c^\top x + \!\!\sum_{\Xi_{\mathrm{opt}}}\!\! p_s Q(x,\omega_s)\quad\text{s.t.}\quad \mathbb{E}_{\Xi_{\mathrm{opt}}}[\Gamma]\le\varepsilon_C,\ \ \text{EENS-feasibility over }\Xi_{\mathrm{verify}}\le \varepsilon_R .
$$
Reliability/resilience enter as **`Ξ_verify` feasibility** (must-pass), not as re-weighted expectation terms (§5.1). Carbon is a constraint on the `Ξ_opt` expectation. Each frontier point is one warm-started run of `𝓛`; ε-constraint (not weighted-sum) recovers non-supported Pareto points on the non-convex surface.

---

## 8. MVP scoping consequence (forward link to design doc)

The review's scope critique is accepted. The first realizable version **removes integer recourse from the Benders inner problem**: LP (SOC-DistFlow) recourse only; switching/restoration/microgrid-formation/mobile-ESS are handled **as oracle verification** (`Ξ_verify` feasibility + structural `OracleFeasibilityCut`), not embedded MILP recourse. NN-as-MILP and L2O are *off* in Phase 1 (certified loop only), added in Phases 2–3 strictly on the proposal path. See `simulation_based_planning_design.md` §7 (revised) and `…_review_response.md`.

---

### Notation
`x` investment · `ω` scenario · `Ξ_opt/verify/audit/stress` scenario roles · `Q_H^opt` ideal recourse (incomputable) · `Q̂_H` simulator achievable cost (UB) · `Q_L` certified relaxation value (LB) · `ê` learned fidelity residual (proposal only) · `M_L/M_C` LB-master / proposal-master · `Gap_SAA = UB − LB^cert` · `𝒮/𝒪/𝓛` oracle / optimizer / loop.

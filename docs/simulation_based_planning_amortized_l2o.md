# Amortized Learning-to-Optimize for the Planning Loop (theory pass)

> Status: **Theory v0.1** · Owner: Matrixeigs · Date: 2026-06-24
> Develops Role 2 (`PrimalPolicy`) of `simulation_based_planning_interfaces.md` along the **imitation / amortized** path. Reuses the bound-sandwich framework of `simulation_based_planning_theory.md`. Governing rule throughout: **ML proposes, the framework certifies** (R1). Nothing here touches `{LB validity, UB validity, finite termination}`.

---

## 1. Setup — planning as an amortized optimization problem

`𝓛` does not solve *one* plan; over a program of work it solves **many instances** — feeders, load-growth cases, carbon targets, budgets. Amortization learns the *solution map* of that family.

- **Instance space** `𝓘`. An instance `i = (G_i, profiles_i, catalog_i, carbon_i, econ_i, Ξ_i⁰)` — a `PlanningCase`. Instances are drawn `i ∼ 𝒟` (the utility's actual fleet of planning studies; in development, IEEE feeders + parametric perturbations).
- **Per-instance objective** (high fidelity, the truth):
$$
F_i(x) \;=\; c_i^\top x \;+\; \sum_{s} p_s\, Q_H^{i}(x,\omega_s),\qquad x\in\mathcal{X}_i .
$$
- **Per-instance optimum** `x^\*(i) = \arg\min_{x\in\mathcal{X}_i} F_i(x)` — or the ε-optimal incumbent `𝓛` certifies.
- **Regret** of a proposed plan `R_i(x) = F_i(x) − F_i(x^\*(i)) \ge 0`.

**Amortized objective (the learning problem):**
$$
\boxed{\;\min_{\theta}\;\; \mathbb{E}_{i\sim\mathcal{D}}\big[\,R_i(\pi_\theta(i))\,\big]\;=\;\text{the *amortization gap*}.\;}
\tag{AO}
$$

`π_θ : 𝓘 → 𝒳_i` is the policy. (AO) is exactly "learning to optimize" / amortized optimization (Amos), specialized to distribution-system expansion. The novelty vs. your prior standalone L2O: **(AO) is never used as the final answer** — `π_θ(i)` is a *warm-start / incumbent* that `𝓛` certifies (§5). A large amortization gap costs iterations, not correctness.

---

## 2. Hypothesis class — a graph policy that transfers across instances

For amortization to *transfer* (not memorize), the policy must be invariant to node relabeling and generalize across network sizes. The natural class is a **graph neural network over the network**, decoding **per candidate site**.

```
G_i  ──►  GNN encoder  ──►  per-node embeddings h_v
                                  │
            per-candidate decoder heads (one per investment element type):
              PV head     :  h_b      → ĉ_pv[b]        (continuous, ≥0)
              ESS head    :  h_b      → (p̂[b], ê[b])   (continuous, ≥0)
              line head   :  h_{(u,v)}→ ŷ_line[uv]∈[0,1] (build logit)
              SOP/VSC head:  h_{site} → ŷ_site, rating
```

- **Node features:** load, peak, capex/embodied-CO₂ at site, existing capacity, voltage class, carbon factor, candidate bounds from `catalog_i`.
- **Edge features:** impedance, length, rating, candidate-line capex.
- **Why GNN:** permutation-invariance + **size-generalization** — a policy trained on 30-bus feeders evaluates a 2000-node system (the report's scale) without retraining the head dimension. This is what makes (AO) an amortizer rather than a lookup table.

### 2.1 𝒳-feasible decoding (postcondition `π_θ(i) ∈ 𝒳_i`)

`𝒳_i` (budget, discrete units, mutual exclusion) is the *cheap* first-stage polytope/lattice — **not** the hard operational feasibility (that's `𝒮`'s job). The decoder must land in `𝒳_i`:

- **Continuous capacities** → bounded by candidate limits via squashing; budget `Σ capex ≤ B` enforced by a differentiable water-filling / softmax-allocation layer.
- **Binary build** → logits → at training, relaxed (sigmoid) for gradients; at inference, **round + cheap repair** onto the `𝒳_i` lattice (a small projection MILP via `𝒪`, or a greedy repair).

Postcondition refinement of the contract: `propose()` returns plans in `𝒳_i`; operational feasibility is **deliberately not** the policy's responsibility.

---

## 3. Training signals — two losses, one schedule

### 3.1 Behavioral cloning (imitation) — needs demonstrations `x^\*(i)`
$$
\mathcal{L}_{\mathrm{BC}}(\theta)=\mathbb{E}_{i}\Big[\underbrace{\textstyle\sum_{\text{bin }j}\mathrm{BCE}(\hat y_j,\,x^\*_j(i))}_{\text{build decisions}} \;+\;\beta\underbrace{\textstyle\sum_{\text{cont }k}(\hat c_k-x^\*_k(i))^2}_{\text{capacities}}\Big].
$$
- **Pros:** stable, supervised, cheap per step.
- **Cons:** (i) needs solved instances; (ii) **multi-optimum / symmetry pathology** — symmetric networks have many interchangeable optima, so regressing to *one* `x^\*` teaches an average of incompatible targets. Mitigated by §3.2 (objective-based, optimum-agnostic) and by predicting a *distribution* (top-k, §5).

### 3.2 Decision-focused (regret) — needs only a **differentiable** objective, *no* demonstrations
Use the surrogate objective from Role 1 (`ê`), which is differentiable:
$$
\tilde F_i^\theta(x)=c_i^\top x+\sum_s p_s\big(\,\underbrace{Q_L^{i}(x,\omega_s)}_{\text{LP value}}+\underbrace{\hat e_\phi(x,\omega_s)}_{\text{NN residual}}\big),
\qquad
\mathcal{L}_{\mathrm{DF}}(\theta)=\mathbb{E}_i\big[\tilde F_i^\theta(\pi_\theta(i))\big].
$$
- `Q_L` is an LP ⇒ differentiable in `x` via the **envelope theorem** (subgradient = optimal duals on the capacity-coupling constraints, returned by `𝒪`).
- `ê_φ` is a ReLU net ⇒ autodiff.
- **Crucially, the `F_i(x^\*)` constant drops out**, so `ℒ_DF` needs **no expert solution** — it directly pushes the policy toward low (surrogate) objective. This sidesteps the symmetry pathology entirely (it rewards *any* good plan).
- **Cons:** quality bounded by surrogate fidelity; can be unstable early.

### 3.3 Recommended schedule (best of both)
$$
\textbf{Pretrain on } \mathcal{L}_{\mathrm{BC}} \;(\text{library of solved instances})\;\longrightarrow\;\textbf{fine-tune on } \mathcal{L}_{\mathrm{DF}}\;(\text{surrogate, no new solves}).
$$
BC gives a stable basin from demonstrations you already pay for; `ℒ_DF` then sharpens toward the objective using only `Q_L`+`ê` — **zero additional `𝒮` calls per gradient step**. This is the concrete synergy between Role 1 (`ê`) and Role 2 (`π_θ`): *the residual surrogate is what makes decision-focused amortization affordable.*

---

## 4. Distribution shift — the amortized DAgger loop

Pure BC suffers covariate shift: `π_θ` leads `𝓛` to instances/regions the demonstrator never solved. The fix is the amortized analogue of **DAgger** (dataset aggregation), where the "expert" is `𝓛` solved to certified optimality:

```
D ← offline-solved instances                       # bootstrap demonstrations
repeat (rounds r = 1,2,…):
    π_θ ← train on D            (ℒ_BC → ℒ_DF)
    sample fresh instances {i} ~ 𝒟
    for each i:  x̂ ← π_θ(i);  run 𝓛 to certified optimum x*(i)   # expert relabel
    D ← D ∪ {(i, x*(i))}                                          # aggregate
until amortization gap on held-out instances stabilizes
```

**Guarantee (online-learning form).** If the per-round training is no-regret over the aggregated dataset, the amortized DAgger policy's expected regret is bounded by the best-in-class regret plus an `O(1/\sqrt{r})` term — i.e. distribution shift is controlled, unlike one-shot BC. (Standard DAgger analysis; here the "trajectory" is the instance stream rather than a control rollout.)

**Demonstration cost & bootstrapping.** Certified solves are expensive, so:
- bootstrap with a **cheap expert** — solve only the relaxation `(P_L)` (no `𝒮` loop) for many instances → abundant approximate demonstrations;
- relabel a *small* fraction with the **full** `𝓛` (high-fidelity) to anchor the policy on truth;
- this two-tier demonstration set mirrors the two-fidelity structure of the whole framework.

---

## 5. How the policy is consumed — and why validity is preserved (R1)

Three usages, in increasing aggressiveness; the first two are **validity-neutral by construction**, the third needs care.

| Usage | Mechanism | Effect on `{LB, UB, termination}` |
|---|---|---|
| **(a) Warm-start master** | `𝒪.set_warm_start(master, π_θ(i))` | none — model unchanged; only the search path differs |
| **(b) Direct incumbents** | evaluate top-k `π_θ(i)` with `𝒮` → instant UB + training data | none — UB only ever *tightens*; `𝒮` certifies (S1) |
| **(c) Restricted-master / trust region** | search a neighborhood of `π_θ(i)` for incumbents | **must isolate**: see below |

**(c) validity rule.** Fixing/penalizing variables toward `π_θ(i)` shrinks the master and can *raise* its optimal value — so a restricted master is **not** a valid lower bound. Therefore: a restricted master may be used **only** as an incumbent (UB) generator, run **alongside** the unrestricted master that supplies the valid LB. Equivalently, use `π_θ(i)` as **branching priority** (which never removes the global optimum). Either way the Theory §5.2 bound sandwich is preserved verbatim. *This is the one place naive L2O-variable-fixing silently breaks correctness; the rule above is mandatory.*

**Scope choice (recommended).** Apply `π_θ` at the **outer instance level** — propose the initial incumbent for `Ξ⁰` — and let the loop's refinement (cuts/CCG) handle the growing scenario set. This avoids conditioning the policy on a *changing* `Ξ` (a moving target). If later you want per-round proposals, condition `π_θ` on summary features of the current `Ξ`.

---

## 6. What can and cannot be claimed (honesty ledger)

| Claim | Status |
|---|---|
| `𝓛` with `π_θ` warm-start returns the **same** certified ε-optimal plan as without it | **Yes** — Theory §5.2 holds verbatim (usages (a),(b),(c-isolated) don't change the model) |
| `π_θ` reduces refinement iterations / `𝒮` calls | **Empirically**; monotone — a warm UB of regret `ρ` lets termination trigger as soon as `LB ≥ opt+ρ−ε` |
| `π_θ` alone yields a feasible/optimal plan | **No** — and we never need it to (R1) |
| Amortization transfers to unseen instances | **Bounded:** `𝔼_{i∼𝒟}[R_i(π_θ)] ≤ \hatℝ_{\text{train}} + O(\sqrt{C(\Theta)/N})` — transfers when `𝒟` is concentrated (similar feeders) and the GNN class generalizes by size-invariance |

The publishable discipline (restated): for `π_θ`, the entry against each of `{LB validity, UB validity, finite termination}` is **"none."** That is the whole point of routing L2O through the certifying loop.

---

## 7. The learning problem, stated as a spec (refines `PrimalPolicy`)

```cpp
struct PrimalPolicy {                              // π_θ : PlanningCase → 𝒳-feasible Plan(s)
  // POST: every returned Plan ∈ 𝒳_i  (first-stage feasible; NOT operationally certified)
  std::vector<Plan> propose(const PlanningCase&, int k) const = 0;   // top-k, diverse

  // training: BC pretrain on demonstrations, then decision-focused fine-tune
  void pretrain_bc(const std::vector<Demo>& demos) = 0;             // Demo=(case, x*)
  void finetune_df(const std::vector<PlanningCase>&,                // no x* needed
                   const RelaxationBuilder&, const ResidualSurrogate&) = 0;

  // amortized-DAgger hook: relabel policy-visited instances with 𝓛's optimum
  void aggregate(const std::vector<Demo>& fresh_expert_labels) = 0;
};
```
**Encoder contract:** GNN over the network graph; per-candidate decode heads aligned to `InvestmentCatalog`; size- and permutation-invariant. **Diversity contract:** top-k proposals must be *diverse* (sampling / DPP / diverse-beam) so `𝒮` evaluates a spread of incumbents, not k near-duplicates.

---

## 8. Open theoretical questions (for the next pass)

1. **Symmetry-aware targets.** Replace BC-to-one-optimum with a set/energy loss invariant to interchangeable plans — or rely solely on `ℒ_DF`. Which generalizes better on symmetric feeders?
2. **Surrogate-induced bias in `ℒ_DF`.** `π_θ` is trained against `Q_L+ê`, not `Q_H`; characterize the gap between the `ℒ_DF`-optimal policy and the true amortizer, and whether DAgger relabeling closes it.
3. **Envelope-theorem subgradients through `Q_L`.** Degeneracy of the LP duals (non-unique) makes the policy gradient noisy; smoothing (proximal/QP regularization of `Q_L`) vs. variance.
4. **Transfer radius.** How far in instance space (load growth, topology change, carbon target) does a trained `π_θ` stay within useful regret before DAgger relabeling is required?
5. **Joint vs. alternating training** of `ê_φ` (Role 1) and `π_θ` (Role 2) — they share the surrogate; co-training dynamics and stability.

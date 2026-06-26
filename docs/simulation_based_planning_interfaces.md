# Interface Contracts — Simulation-Based Optimal Planning (type-level spec)

> Status: **Spec v0.2** (revised after adversarial review) · Owner: Matrixeigs · Date: 2026-06-24
> Companion to `simulation_based_planning_theory.md` (v0.2). This document fixes the **type-level contracts** the three modules expose so the Planning loop `𝓛` can drive the optimization operator `𝒪` (MIPSolvers) and the simulation oracle `𝒮` (hacdcpf). Signatures are C++-flavored but normative as *contracts* (pre/post/invariants), not implementations.
>
> **v0.2 changes:** three-state oracle with `CertificateValidity`; `op_cost` is a *materialized-trajectory* cost (not optimal recourse); `Full` fidelity = trajectory-consistency standard; duals/rays are *tolerance-valid*; the cut path is gated by a validator; the interaction trace uses the **two-master** (LB vs proposal) architecture of theory §4.

---

## 0. Design rules (normative)

- **R1 — Validity never depends on ML or on the surrogate.** Lower bounds come only from the relaxation solved by `𝒪`; upper bounds come only from `𝒮`. Any learned component is a *proposal* that is subsequently certified. (See §6.)
- **R2 — The relaxation invariant (R) is a compile-time/contract obligation of the relaxation builder, not a runtime hope.** Every emitted constraint is tagged `exact | relaxation | dropped`; a builder may never emit a constraint that is *not* implied by the true operating set `𝒴_H`.
- **R3 — Infeasibility certificates are *graded* (`CertificateValidity`).** `𝒮` returns one of three statuses; only a `Validated` certificate may produce a validity-critical cut. `Heuristic`/`None` certificates and `Indeterminate` returns may **never** modify `M_L` or the bounds (theory §2, §4.3). Requiring a clean certificate from every nonconvex AC failure is unrealistic — so the contract *grades* certificates rather than demanding them.
- **R4 — `𝒮` is reproducible up to tolerance:** `evaluate` is a deterministic function of `(plan, scenario, seed, fidelity)` to within a declared numerical tolerance. Duals/rays from `𝒪` are likewise *tolerance-valid*, not exact reals.
- **R5 — Module boundaries carry only the boundary types of §1.**
- **R6 — `op_cost` is the cost of a *materialized feasible trajectory*, not an optimal recourse value** (theory §1.2, $\widehat Q_H \ne Q_H^{\mathrm{opt}}$). It is a valid **upper-bound** ingredient only; it never bounds from below.
- **R7 — LB comes only from the pure-relaxation master `M_L`** (theory §4). No surrogate-corrected solution, no learned value ever writes `LB`.

---

## 1. Boundary data types (the shared vocabulary)

```cpp
namespace planning {

// ── x : the first-stage decision ─────────────────────────────────────────
// Owned by 𝓛. Must be expressible two ways: as fixed variable values for 𝒪,
// and as a concrete network for 𝒮.
struct Plan {
  // dense, canonical encoding of all investment variables (siting+sizing)
  std::vector<double>  cont;          // continuous part  (capacities, ratings)
  std::vector<int>     disc;          // integer/binary part (units, build flags)

  // contract: materialize the plan into a simulate-able network for 𝒮
  hacdcpf::HybridPowerSystem as_built(const InvestmentCatalog&) const;   // → 𝒮 input
  // contract: stable key for caching / no-good cuts / surrogate samples
  PlanKey key() const;                                                   // hashable
};

// ── ω : a scenario (tagged union of the three modes) ─────────────────────
enum class Mode { TypicalDay, Contingency, ResilienceEvent };
struct Scenario {
  Mode                 mode;
  double               weight;        // p_s   (π_s or ω_s)
  uint64_t             seed;          // for R4 reproducibility
  // mode-specific payload (profiles / outage set / event trajectory)
  ScenarioPayload      data;          // consumed by BOTH 𝒮 and the relaxation builder
};
using ScenarioSet = std::vector<Scenario>;   // Ξ

// ── m : performance metrics returned by 𝒮 ────────────────────────────────
struct Metrics {
  // R6: cost of a SINGLE materialized trajectory satisfying ALL physical
  // constraints for this scenario. UB ingredient only; ≠ optimal recourse.
  double op_cost;
  bool   from_materialized_trajectory; // false ⇒ NOT usable as a UB cost ingredient
  double carbon_tco2;                  // MUST be derived from the same trajectory
  // statistical estimates — usable for Ξ_verify feasibility / reporting, NOT as UB cost:
  double eens_mwh, lole_hr, saidi_min, resilience_index;
  std::vector<double> nodal_voltage;   // AC feasibility witness of the trajectory
  std::vector<double> recovery_curve;
};

// ── ν : the graded feasibility verdict + certificate (R3) ────────────────
enum class Status { Feasible, Infeasible, Indeterminate };   // three states
enum class CertificateValidity { Validated, Heuristic, None };// only Validated → M_L cut
struct ViolationCertificate {
  enum Kind { VoltageBound, ConverterLimit, ThermalLimit,
              UnservedCritical, IslandInfeasible, CarbonCap } kind;
  CertificateValidity validity;       // gate for validity-critical cuts (R3)
  std::vector<int>    loci;           // buses/branches/zones implicated
  double              margin;         // signed
  std::optional<StructuralWitness> witness;  // present ⇒ generalizable structural cut
};
struct Verdict { Status status; std::optional<ViolationCertificate> cert; };

// Oracle-status → action table (theory §2, §6 algorithm). NORMATIVE:
//   Feasible (from_materialized_trajectory)      → update UB ; no cut
//   Infeasible + cert.validity==Validated        → add OracleFeasibilityCut to M_L
//   Infeasible + cert.validity∈{Heuristic,None}  → cut to M_C only (proposal), never M_L
//   Indeterminate                                → no UB, no M_L cut; escalate/retry;
//                                                  if unresolved → mark case uncertified

} // namespace planning
```

---

## 2. `𝒪` — the Optimization Operator (MIPSolvers contract)

`𝒪` is **already** `mipsolvers::aml` + `engine::branch_and_cut`. The spec below states exactly what `𝓛` requires of it; most already exists, the *callback* and *dual/ray* guarantees are the load-bearing additions.

```cpp
namespace planning::opt {                // thin contract over mipsolvers

// 2.1 Model handle (built by 𝓛 via the AML; 𝒪 owns the solve)
struct Model;                            // opaque; wraps aml::Model

// 2.2 What a solve must return — duals and rays are MANDATORY for cuts
struct SolveResult {
  enum { Optimal, Feasible, Infeasible, Unbounded, TimeOut } status;
  double               objective;        // master obj  → contributes to LB
  double               bound;            // best dual bound (gap = obj−bound)
  std::vector<double>  primal;           // x̂  (master) or ŷ (subproblem)
  std::vector<double>  dual;             // λ : REQUIRED for LP optimality cuts
  std::vector<double>  ray;              // dual ray : REQUIRED for feasibility cuts
};

// 2.3 The driver interface 𝓛 calls
struct Operator {
  // exact solve of any algebraic model handed in
  SolveResult solve(Model&, const SolveOptions&) = 0;

  // CUT / COLUMN CALLBACK CONTRACT  (branch-and-cut lazy-constraint hook).
  // 𝒪 calls `gen` at each integer-feasible incumbent x̂ of the master; 𝓛
  // returns cuts to add lazily. If callbacks are unavailable, 𝓛 falls back to
  // an outer cutting-plane loop (re-solve master each round) — same cuts.
  using LazyCutGen = std::function<std::vector<Cut>(const std::vector<double>& xhat)>;
  void register_lazy_cuts(Model&, LazyCutGen gen) = 0;

  // CCG: add a whole recourse block (new scenario column) between solves
  void add_column(Model&, const RecourseColumn&) = 0;

  // L2O / warm-start contract: seed the search with a proposed incumbent
  void set_warm_start(Model&, const std::vector<double>& x_proposed) = 0;
};
} // namespace planning::opt
```

**Contract obligations on `𝒪`:**
- (O1) For **LP** subproblems, `dual` and `ray` are **tolerance-valid** and consistent with the returned primal (strong duality up to the solver's feasibility/optimality tolerance). These produce Benders optimality/feasibility cuts; cuts are slackened by the tolerance to remain valid (see cut validator below).
- (O2) For **MILP `M_L`**, `solve` returns a valid dual `bound`; **`LB` is read from this bound** (theory §4.1), never from a primal point of `M_C`.
- (O3) `register_lazy_cuts` cuts are added **without invalidating** previously explored nodes (true lazy constraints).
- (O4) Determinism under fixed seed/threads up to a reported tolerance.
- (O5) **Cut validator:** every cut handed to `𝒪` for `M_L` passes a validator that checks it excludes no point of the certified relaxation (registry, theory §3.1) — invalid/heuristic cuts are routed to `M_C` only.

---

## 3. `𝒮` — the Simulation Oracle (hacdcpf contract)

`𝒮` wraps the existing simulators behind **one** uniform entry. The novelty vs. today's API is (a) the *certificate* (R3) and (b) the *fidelity selector* for escalation.

```cpp
namespace planning::sim {

enum class Fidelity {
  Snapshot,    // single AC-OPF / PF feasibility gate         (cheapest)
  Annual,      // solve_annual_production_simulation           (cost+ENS)
  Reliability, // run_*_reliability                            (EENS/SAIDI)
  Resilience,  // run_distribution_resilience_mip_assessment   (recovery)
  Full         // all of the above + run_carbon_analysis       (authoritative)
};

struct OracleReturn {                    // the realization of 𝒮(x,ω)
  Verdict   verdict;                      // ν   (status + graded certificate)
  Metrics   metrics;                      // m   (op_cost = materialized-trajectory cost, R6)
  double    wall_time_s;                  // for fidelity-escalation budgeting
};

struct Oracle {
  // 𝒮 : (x, ω, fidelity) ↦ (status, op_cost, verdict, metrics)   — THREE states
  // POST: Feasible      ⇒ op_cost is the cost of a materialized feasible trajectory
  //                        (from_materialized_trajectory==true); valid UB ingredient,
  //                        NOT asserted equal to Q_H^opt (theory §1.2, R6).
  //       Infeasible    ⇒ op_cost=+∞; verdict.cert.validity ∈ {Validated,Heuristic,None}.
  //       Indeterminate ⇒ no usable cost, no cut; caller escalates/retries (S5).
  OracleReturn evaluate(const Plan&, const Scenario&, Fidelity) = 0;

  // batched form: 𝓛 evaluates a plan across Ξ (parallel inside hacdcpf)
  std::vector<OracleReturn> evaluate(const Plan&, const ScenarioSet&, Fidelity) = 0;
};
} // namespace planning::sim
```

**Contract obligations on `𝒮`:**
- (S1) **Upper-bound honesty via materialized trajectory (R6):** a `Feasible` verdict's `op_cost` is the cost of a *single concrete operation trajectory* that satisfies all physical constraints for that scenario, with `from_materialized_trajectory==true`. It is achievable, hence a valid **UB** ingredient — but it is **not** asserted to equal the optimal recourse `Q_H^opt` (theory §1.2). Statistical/post-processed metrics set `from_materialized_trajectory=false` and are excluded from the UB cost.
- (S2) **Graded certificates (R3):** an `Infeasible` verdict *should* carry a `ViolationCertificate` with a `CertificateValidity`. Only `Validated` licenses an `M_L` cut; absence/`None`/`Heuristic` is permitted (nonconvex failures may be uncertifiable) and routes to `M_C` or to `Indeterminate` handling.
- (S3) **Monotone fidelity:** feasible at `Full` ⇒ feasible at `Snapshot`. `Full` is defined by the **trajectory-consistency standard**: (i) one materialized dispatch/restoration trajectory per scenario; (ii) it satisfies all physical constraints; (iii) carbon/loss metrics derive from it; (iv) any purely statistical module (MC reliability) is flagged and not a UB cost ingredient. (Theory §2, R-F.)
- (S4) **Reproducibility up to tolerance (R4).**
- (S5) **Three-state honesty:** `Indeterminate` (solver failure / nonconvergence / timeout) is returned rather than a guessed Feasible/Infeasible. The loop handles it per the status→action table; finite-termination claims exclude it.

---

## 4. The Relaxation Builder (the bridge `𝓛` owns; the (R)-invariant lives here)

This is the most theory-critical contract: it is where property (R) is *enforced by construction*. It consumes `𝒪`'s AML to emit `Q_L`, and it manufactures cuts after each subproblem solve.

```cpp
namespace planning {

enum class ConstraintTag { Exact, Relaxation, Dropped };   // R2 audit tag
enum class LbGrade { Certified, ModelInternal };           // theory §3.2

struct RelaxationBuilder {
  // Build the low-fidelity recourse Q_L(x,ω) as an AML submodel.
  // x enters as FIXED PARAMETERS (the master sets them); coupling constraints
  // (operation ≤ capacity(x)) reference them.
  // INVARIANT (R, theory §3): every emitted constraint is tagged; each
  //   `Relaxation` tag CITES a registry lemma proving it is an OUTER
  //   approximation of its high-fidelity counterpart; a template that cannot
  //   be so certified MUST be `Dropped` (which preserves the relaxation),
  //   never approximated-in-place. Q_L ≤ Q_H^opt holds ONLY when lb_grade()
  //   == Certified.
  opt::Model build_recourse(const Plan& x_fixed, const Scenario&) = 0;

  // After solving build_recourse at x̂, manufacture the cut valid for the master:
  //   • LP recourse        → OptimalityCut / FeasibilityCut from duals/ray (O1)
  //   • MILP recourse       → IntegerLShapedCut (Laporte–Louveaux); duals INVALID
  Cut make_benders_cut(const Scenario&, const opt::SolveResult& sub) = 0;

  // self-audit: tag census; CI asserts every `Relaxation` cites a registry lemma.
  std::map<ConstraintTag,int> audit() const = 0;
  // Certified ⇒ M_L bound is a valid LB on v*_Ξ; ModelInternal (e.g. LinDistFlow)
  // ⇒ M_L bound is only relaxation-model-internal (theory §3.2). Gates the claim.
  LbGrade lb_grade() const = 0;
};
} // namespace planning
```

---

## 5. The cut / column algebra (what crosses back into the master)

```cpp
namespace planning {

// θ_s ≥ α + βᵀx   — convex (LP) recourse, from duals λ
struct OptimalityCut    { int s; double alpha; std::vector<double> beta; };

// γᵀx ≥ δ          — exclude x that make subproblem s infeasible, from ray
struct FeasibilityCut   { int s; std::vector<double> gamma; double delta; };

// Laporte–Louveaux: θ_s ≥ (Q̂−L)(1 − Σ_{j∈S}(1−x_j) − Σ_{j∉S}x_j) + L
// valid for BINARY x with INTEGER recourse (where Benders duals do not apply)
struct IntegerLShapedCut{ int s; double Qhat, L; std::vector<int> support; };

// CCG: a whole new scenario block appended to the master (variables+coupling)
struct RecourseColumn   { Scenario omega; /* aml vars+constraints handle */ };

// derived from 𝒮's ViolationCertificate (NOT from a dual).
//   • default  : no-good cut excluding exactly x̂ (finite conv. for binary x, A3)
//   • structural: a valid inequality implied by `witness`
//                 (e.g. Σ_{e∈Z} E_ess ≥ τ·peak_Z)  — generalizes, prunes faster
struct OracleFeasibilityCut { std::vector<double> a; double b; bool structural; };

using Cut = std::variant<OptimalityCut, FeasibilityCut,
                         IntegerLShapedCut, OracleFeasibilityCut>;
} // namespace planning
```

**Validity conditions (must be checked, not assumed):**
| Cut | Valid when | Source |
|---|---|---|
| `OptimalityCut` | recourse is an LP (convex) | subproblem duals (O1) |
| `FeasibilityCut` | LP subproblem infeasible | dual ray (O1) |
| `IntegerLShapedCut` | `x` binary, recourse MILP | objective at fixed `x̂` |
| `OracleFeasibilityCut` (no-good) | `x` binary | `𝒮` certificate (S2) |
| `OracleFeasibilityCut` (structural) | `witness` implies a valid inequality for `(P_H)` | `𝒮` certificate (S2) |

---

## 6. Where machine learning enters — and how it rehabilitates L2O

**Governing principle (restates R1): _ML proposes, the framework certifies._**
Every learned component sits on the *efficiency* path, never the *correctness* path. The bound sandwich (`LB` from `𝒪` on the pure relaxation, `UB` from `𝒮`) certifies whatever ML proposes. This is the exact reason your earlier **learning-to-optimize (L2O)** attempt is *safe and useful here* though it failed standalone: pure L2O cannot guarantee feasibility or optimality of the plan it emits — but inside this loop its output is just an incumbent that `𝒮` verifies and `𝒪` bounds. **The framework supplies the guarantee L2O lacks.**

### 6.1 The five ML roles, each mapped to a formal object and a contract

| # | ML role | Realizes / attaches to | Contract type | Risk if wrong | Priority |
|---|---|---|---|---|---|
| 1 | **Residual surrogate** `ê_θ(x,ω) ≈ Q_H − Q_L` | corrects `Q_L`→`Q_C` inside `𝒪` | `ResidualSurrogate` (§6.2) | slower, never invalid | **High** |
| 2 | **L2O primal policy** `π_θ(case) → Plan` | warm-start / incumbent for `𝒪` & `𝒮` | `PrimalPolicy` (§6.3) | wasted eval, never invalid | **High** (reuses your L2O) |
| 3 | **Scenario active-learner** `σ_θ → critical ω` | feeds refinement (Thy §4.1) | `ScenarioSelector` (§6.4) | slower convergence | Med |
| 4 | **Feasibility classifier** `φ_θ(x,ω) → P[feasible]` | pre-screen before `𝒮` | `FeasibilityScreen` (§6.4) | wasted/skipped eval* | Med |
| 5 | **Learned branching / cut-selection** | inside `𝒪` (solver) | solver-internal | slower, never invalid | Low |

\* Role 4 must **never** veto an evaluation that would have produced a UB/certificate; it may only *reorder* evaluations. Its false-negatives cost time, not validity.

### 6.2 Residual surrogate contract (Role 1 — the `ê` of Theory §4.2)

```cpp
struct ResidualSurrogate {                        // ê_θ : (Plan,Scenario) → metrics
  std::vector<double> predict(const Plan&, const Scenario&) const = 0;
  void fit(const std::vector<Sample>& s) = 0;     // Sample = (Plan,Scenario,OracleReturn)

  // MUST be MILP-representable so 𝒪 can embed it in the master:
  // emits ReLU big-M constraints for the network into the AML model.
  void embed(opt::Model&, /*x vars*/, /*aux θ vars*/) const = 0;   // ReLU-as-MILP
  // contract: piecewise-linear architecture only (ReLU/maxout); bounded weights
  // for tight big-M. Used ONLY to guide incumbents (R1) — LB stays from Q_L.
};
```

### 6.3 L2O primal policy contract (Role 2 — your prior work, made safe)

```cpp
struct PrimalPolicy {                              // π_θ : PlanningCase → Plan(s)
  // POST: output need NOT be feasible or optimal. It is a PROPOSAL.
  std::vector<Plan> propose(const PlanningCase&, int k) const = 0;  // top-k plans
  void train(const std::vector<SolvedInstance>& history) = 0;       // imitation/RL
};
// 𝓛 uses it as: (a) set_warm_start(master, π_θ(case)) — accelerate 𝒪;
//               (b) evaluate proposals directly with 𝒮 → instant UB & training data.
// Because 𝒮 certifies and 𝒪 bounds, a bad policy only wastes time (R1).
```

> Two viable L2O formulations, both fit this contract:
> - **Imitation / amortized optimization:** train `π_θ` to mimic plans `𝓛` found on past instances → predict good plans on new instances (warm-start). Lowest risk, immediate payoff.
> - **RL over the loop:** treat (propose plan → get `𝒮` reward) as an MDP; learn a policy that maximizes certified objective. Higher variance; only worthwhile once the loop and surrogate are solid.

### 6.4 Selector / screen contracts (Roles 3–4)

```cpp
struct ScenarioSelector {                          // σ_θ : (Plan,Ω) → ranked Ξ′
  ScenarioSet propose_critical(const Plan&, const ScenarioPool&, int k) const = 0;
  // contract: a RANKER of which ω to evaluate/refine next; 𝒮 confirms. Never the
  // sole source of feasibility — A3's finite set still bounds correctness.
};
struct FeasibilityScreen {                          // φ_θ : (Plan,Scenario)→[0,1]
  double p_feasible(const Plan&, const Scenario&) const = 0;
  // contract: REORDER 𝒮 calls only; may not skip an evaluation needed for a bound.
};
```

### 6.5 Recommendation

Build ML in this order, gated on the loop working without it first:
1. **Residual surrogate `ê` (Role 1).** Highest leverage, already in the theory, embeddable, and validity-neutral. Train on the `𝒮` samples the loop generates anyway.
2. **L2O primal policy as warm-start (Role 2, imitation flavor).** Directly reuses your prior learning-to-optimize work; now it can never produce an invalid plan because `𝒮`/`𝒪` certify it. Biggest reuse of existing effort for least new risk.
3. **Scenario active-learning (Role 3).** Tightens the refinement loop on large `Ω`.
4. Defer Roles 4–5 until profiling says the bottleneck is `𝒮`-screening or solver node-count.

The discipline that makes all of this publishable rather than ad-hoc: **state, for every learned block, which of `{LB validity, UB validity, finite termination}` it could affect — the answer must always be "none."**

---

## 7. Interaction trace — `𝓛` driving `𝒪` and `𝒮` (one iteration, two-master)

The **LB and proposal masters are distinct** (theory §4). `M_L` is pure relaxation and is the *only* writer of `LB`; `M_C` carries `ê`/warm-starts and only *proposes*.

```
# ---- M_L : LB master (pure certified relaxation; ONLY source of LB) ---------
gen_L(x̂) = for ω in Ξ_opt:
              sub ← RelaxationBuilder.build_recourse(x̂, ω)      # registry-certified (§3.1)
              r   ← 𝒪.solve(sub)                                 # tolerance-valid duals (O1)
              c   ← RelaxationBuilder.make_benders_cut(ω, r)
              if validator.ok(c):  yield c into M_L              # else → M_C only (O5)
𝒪.register_lazy_cuts(M_L, gen_L)
resL ← 𝒪.solve(M_L)
LB   ← resL.bound                       # dual bound of M_L  (theory §4.1) — NEVER from M_C

# ---- M_C : proposal master (corrected surrogate / L2O) — NOT a bound --------
𝒪.set_warm_start(M_C, π_θ(case))                                # L2O warm-start (optional)
x̂    ← 𝒪.solve(M_C).primal              # candidate only; objective DISCARDED for bounds

# ---- 𝒮 : verify the candidate; three-state handling ------------------------
feasible_all ← true
for ω in (Ξ_opt ∪ Ξ_verify) ranked by σ_θ:                      # Role 3 (optional)
    o ← 𝒮.evaluate(x̂, ω, Fidelity::Snapshot→…→Full)             # escalate (S3)
    switch o.verdict.status:
      Feasible:        if !o.metrics.from_materialized_trajectory: feasible_all=false  # R6
                       else accumulate o.metrics.op_cost                               # UB
      Infeasible:      feasible_all=false
                       if o.verdict.cert.validity==Validated:
                            add OracleFeasibilityCut → M_L       # validity-critical (§4.3)
                       else add to M_C only
      Indeterminate:   feasible_all=false ; escalate/retry ; if unresolved mark uncertified
if feasible_all:                                                # only then a valid UB
    UB ← min(UB, c·x̂ + Σ_{Ξ_opt} p_s·op_cost_s) ; if improved x*←x̂
𝓛.surrogate.fit(new Samples)                                    # ê retrain — proposal only
report Ξ_audit statistics SEPARATELY (never merged into UB−LB)  # theory §5.2
if UB − LB ≤ ε and no new Validated violation:  return x*, Gap_SAA, audit_report
```

---

## 8. What is genuinely *new* work vs. already present

| Contract | Status today | Work for Planning module |
|---|---|---|
| `𝒪.solve` + duals/ray | present (`aml`/HiGHS) | confirm ray exposure for feasibility cuts |
| `𝒪.register_lazy_cuts` | partial (`branch_and_cut`) | expose lazy-constraint callback to `𝓛` |
| `𝒮.evaluate` uniform entry | simulators exist, no façade | **new thin façade** over annual/reliab/resil/carbon |
| `ViolationCertificate` (S2) | metrics yes, certificate no | **new** — structured cause + witness |
| `RelaxationBuilder` + (R) audit | `lindistflow_builder` exists | **new** — investment coupling + tag audit |
| Cut algebra incl. integer L-shaped | — | **new** in Planning module |
| ML contracts (§6) | `NeuralNetworkMILP.jl` proto | port `ê`; wrap prior L2O as `PrimalPolicy` |

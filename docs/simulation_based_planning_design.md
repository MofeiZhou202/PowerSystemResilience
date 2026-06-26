# Simulation-Based Optimal Planning of Hybrid AC/DC Distribution Systems — Design Document

> Status: **Roadmap v0.2** · Owner: Matrixeigs · Date: 2026-06-24
> Scope: a planning layer built **on top of** the existing `hacdcpf` simulation engine, realizing the low-carbon / high-reliability planning vision from `LowCarbonDistributionSystemPlanning` and the technical report.
>
> **This document is the roadmap/architecture only. It carries NO theoretical guarantees** — all bound/optimality/termination claims live in `simulation_based_planning_theory.md` (v0.2), and the contracts in `…_interfaces.md` (v0.2). v0.2 narrows scope after adversarial review: the realizable system is a **bound-certified, simulation-verified, ML-accelerated** planner, *not* a full high-fidelity-global-optimal / unconditionally-finite / cross-scale-generalizing system. The MVP (§7) is the certified loop with **no ML and no integer recourse**.

---

## 0. The Core Idea (read this first)

We already own two of the three pieces required for planning, and they have never been wired together:

| Piece | Where it lives today | What it is |
|---|---|---|
| **Optimizer concept** | `LowCarbonDistributionSystemPlanning/` (Julia) | Two-stage stochastic MILP (invest → operate), Benders / CCG, NN-surrogate-as-MILP, Bayesian scenario refinement — but on a *simplified DC / LinDistFlow* operation model. |
| **High-fidelity evaluator** | `hacdcpf` (this C++ repo) | Hybrid AC/DC PF/OPF, unit commitment, 8760-h annual production, multi-year lifecycle (aging), Monte-Carlo & 3-stage reliability, resilience restoration MIP, carbon-flow tracing, scenario generation. |
| **Vision / requirements** | `Low-carbon System Planning Technical Report` | Carbon–reliability coupling; multi-voltage flexible interconnection; 8 planning elements × 7 planning modes; closed-loop "ML-driven full-cycle simulation verification → decision". |

The report repeatedly names the missing capability: **"机器学习驱动的规划方案全周期仿真推演验证"** (ML-driven full-lifecycle simulation verification of planning schemes) and **"推演-决策闭环优化"** (simulate–decide closed-loop optimization). That capability is precisely the bridge between the *simplified* Julia planner and the *high-fidelity* `hacdcpf` simulators.

**Definition — Simulation-Based Optimal Planning (this project).**
> A closed loop in which a tractable **investment optimizer** proposes candidate plans, the **high-fidelity `hacdcpf` simulators** evaluate each plan across a scenario space (production cost, carbon, reliability, resilience), and the gap between the optimizer's internal (low-fidelity) operation model and the simulator's verdict is closed by (a) **scenario refinement** (add the scenarios the plan actually fails) and (b) a **surrogate** trained on simulator outputs and embedded back into the optimizer. Iterate until the optimizer's plan and the simulator's verdict agree.

This is the distinguishing thesis of the project: planning decisions are not validated by the same simplified model that produced them (the classic weakness of MILP-only expansion planning), but by an independent, higher-fidelity hybrid AC/DC engine — and that verification is *fed back* into the optimizer rather than left as a post-hoc check.

```
          ┌─────────────────────────── CLOSED LOOP ───────────────────────────┐
          │                                                                    │
   ┌──────▼───────┐   plan x*    ┌──────────────────┐  metrics   ┌────────────┴───────┐
   │  INVESTMENT  │ ───────────► │   HIGH-FIDELITY  │ ─────────► │   VERIFIER /       │
   │  OPTIMIZER   │              │   hacdcpf SIMS   │            │   SCENARIO REFINER │
   │ (2-stage MILP│ ◄─────────── │  PF/OPF/UC/annual│ ◄───────── │  + SURROGATE FIT   │
   │  + Benders)  │  new cuts /  │  reliab/resil/CO2│  worst     └────────────────────┘
   └──────────────┘  scenarios / └──────────────────┘  scenarios
                     surrogate
```

---

## 1. What we are planning (decision space)

Taken from the Julia prototype + the report's "8 planning elements". The first-stage (here-and-now) investment variables:

| Element (报告术语) | Decision variable | Type | Phase |
|---|---|---|---|
| Distributed PV (分布式光伏) | `C_pv[b]` capacity at bus `b` | cont. + integer units | 1 |
| Wind (风电) | `C_wind[b]` | cont. + integer units | 1 |
| Substation / station ESS (台区储能) | `P_ess[b]`, `E_ess[b]` (power + energy) | cont. + integer units | 1 |
| AC/DC converter / energy router (能源路由器) | `C_vsc[k]` rating at candidate site `k` | cont. + binary site | 2 |
| MV/LV flexible interconnection (柔性互联, SOP) | `y_sop[l] ∈ {0,1}` + rating | binary + cont. | 2 |
| MV lines / reconductoring (中压线路) | `y_line[l] ∈ {0,1}` build/upgrade | binary | 2 |
| Mobile / shared ESS (移动·共享储能) | `P_sess`, `E_sess`, location `I_m ∈ {0,1}` | cont. + binary | 3 |
| Microgrid formation (微电网) | partition + grid-forming assignment | binary | 3 |

Second-stage (recourse / operation) variables — per scenario `s`, per time step `t` — are **exactly the variables the `hacdcpf` operation models already expose**: generator/renewable/ESS dispatch, charge/discharge, converter rectify/invert power, branch flows, voltages, load shedding, switch states. This is the key reuse: the recourse problem *is* a parameterized `hacdcpf` operation model.

---

## 2. Mathematical formulation

### 2.1 Two-stage stochastic program (carbon–reliability coupled)

$$
\min_{x \in \mathcal{X}}\; \underbrace{c^\top x}_{\text{annualized investment}}
\;+\; \underbrace{\sum_{s \in \mathcal{S}} \pi_s\, Q(x,\xi_s)}_{\text{expected operation cost}}
\;+\; \underbrace{\text{VOLL}\sum_{s} \omega_s\, \text{EENS}_s(x)}_{\text{reliability / resilience penalty}}
$$

subject to investment feasibility `Ax ≤ b`, discretization `C = Δ·I`, and the **carbon constraint** that defines the "low-carbon" character:

$$
\sum_{s}\pi_s\, \text{Carbon}_s(x) \;\le\; (1-\eta)\, \text{Carbon}_{\text{base}},
\qquad \eta = \text{reduction target on the city decarbonization pathway }\lambda.
$$

The recourse value `Q(x,ξ_s)` is the optimal operation cost of scenario `s` given plan `x` — evaluated at the chosen fidelity (see §3). The report's multi-objective form `min f(x) + ρ·Carbon + (1-ρ)·Reliability` is recovered by moving carbon/reliability between objective and constraint via the weight `ρ` / target `η`.

### 2.2 Recourse (operation) constraints — provided by `hacdcpf`

Per scenario and step: nodal power balance (AC & DC bus forms with converter coupling), branch/converter limits, ESS SoC dynamics with cyclic boundary, renewable availability `≤ Δ_pv·C_pv`, voltage bounds (LinDistFlow squared-voltage), priority load shedding. These are emitted today by `lindistflow_builder` (radial AC), `dc_opf_builder`, `scuc_builder` (UC), and the `parity` IPM (full hybrid AC/DC). The planner **adds investment-coupling constraints** of the form "operation variable ≤ installed capacity" via big-M / direct bounds.

### 2.3 Scenario space (unified, from the Julia framework)

$$
\Xi = \Omega_{\text{typical}}\;\cup\;\Omega_{\text{reliability}}\;\cup\;\Omega_{\text{resilience}}
$$

- `Ω_typical`: representative days from clustering of 8760-h profiles (k-means/k-medoids + LDC), with probabilities `π_s`.
- `Ω_reliability`: N-1 / N-k contingencies with weights `ω_s`.
- `Ω_resilience`: extreme events (typhoon, multi-fault, equipment derating) over a recovery horizon.

`hacdcpf::generate_scenarios` already produces all three families (`Regular`, `Reliability`, `Resilience`/typhoon). The planner consumes its output directly.

---

## 3. Three coupling fidelities (the heart of "simulation-based")

The planner can evaluate `Q(x,ξ_s)` at three fidelities. The design supports all three and **escalates** through them in the closed loop.

### Fidelity A — Embedded low-fidelity operation model (fast, inside the MILP)
The recourse is a LinDistFlow / DC-network LP built by extending `lindistflow_builder` / `scuc_builder`. This is what the Julia prototype does. Fast, gives exact duals for Benders, but optimistic (linearized, no AC voltage/converter detail). **Used for the optimization sweep.**

### Fidelity B — High-fidelity simulation verification (slow, outside the MILP)
A candidate plan `x*` is frozen into a `HybridPowerSystem` and evaluated with the real simulators:

| Metric | `hacdcpf` entry point | Returns |
|---|---|---|
| Annual production cost & energy | `solve_annual_production_simulation` | cost, gen MWh, ENS MWh, per-step dispatch |
| Multi-year aging / replacement | `run_lifecycle_simulation` | SOH, replacements, annual carbon w/ error bounds |
| Reliability indices | `run_nonsequential_mc` / `run_distribution_fmea` / `run_three_stage_reliability` | EENS, LOLE, SAIFI, SAIDI, nodal EENS, critical components |
| Resilience / restoration | `run_distribution_resilience_mip_assessment` | recovery curves, resilience index, max load shed |
| Carbon footprint | `run_carbon_analysis` on each dispatch snapshot | per-load/branch/generator CO₂ attribution |
| AC feasibility check | `solve_ac_opf` / `solve_power_flow` | true voltages, converter limits, infeasibility hints |

This is the **independent verdict** the MILP never sees internally.

### Fidelity C — Surrogate (learned from B, embedded into the *proposal* master only)
Sample plans → run Fidelity-B simulators → train a surrogate `ê(x) ≈ {carbon, EENS, deficit, cost}` → embed `ê` via the **NN-as-MILP (ReLU big-M) encoding** prototyped in `simulation/NeuralNetworkMILP.jl`. **Critical (theory §4):** the surrogate enters only the *proposal* master `M_C`, never the *bound* master `M_L` — the lower bound is always read from the pure relaxation, so the surrogate can never corrupt it. **Engineering caveat (review §9):** embedding a ReLU net per scenario/candidate can blow up the master (binary count, loose big-M). Mitigate with tight bound propagation, few hidden units, surrogate only on the corrected objective, and an off-by-default flag — measure node-count before committing.

### The closed loop (ties A/B/C together)
```
1. Build Ξ via generate_scenarios; cluster typical days; enumerate N-1; sample resilience.
2. Solve 2-stage MILP at Fidelity A (Benders) → candidate plan x*.
3. Freeze x* → HybridPowerSystem; run Fidelity-B simulators over Ξ_full.
4. Compute the gap: does x* satisfy carbon/EENS/voltage under high fidelity?
     - If a scenario ω makes x* infeasible/unacceptable → add ω to Ξ (scenario refinement, CCG-style)
       and/or add a feasibility cut to the master.
     - Periodically retrain surrogate g on accumulated (x, B-metrics) samples; refresh Fidelity-C terms.
5. Repeat 2–4 until |UB−LB| < ε AND high-fidelity verdict agrees (no new violated scenario).
6. Report Pareto set over ρ (cost ↔ carbon ↔ reliability).
```

---

## 4. Software architecture

Follows the repo's existing layering (`rich → validate → canonical projection → model assembly → solve → unproject`). The planner is a **new top layer** that orchestrates existing layers; it does not replace them.

```
include/hacdcpf/planning/                 src/planning/
  planning_case.hpp        ─ data model    planning_case.cpp
  investment_model.hpp     ─ 1st-stage     investment_model.cpp      (extends aml builders)
  recourse_model.hpp       ─ 2nd-stage     recourse_model.cpp        (wraps lindistflow/scuc/dc_opf)
  benders.hpp              ─ decomposition benders.cpp                (reuses engine/branch_and_cut)
  ccg.hpp                  ─ robust/refine ccg.cpp
  scenario_set.hpp         ─ Ξ management  scenario_set.cpp          (wraps generate_scenarios + clustering)
  evaluator.hpp            ─ Fidelity B    evaluator.cpp             (calls annual/reliability/resilience/carbon)
  surrogate.hpp            ─ Fidelity C    surrogate.cpp             (NN-as-MILP encoding)
  planning_loop.hpp        ─ closed loop   planning_loop.cpp         (orchestrator)
  planning_result.hpp      ─ outputs       planning_result.cpp
```

### 4.1 Core data structures (sketch)

```cpp
namespace hacdcpf::planning {

// Candidate sites + bounds for every investment element.
struct InvestmentCatalog {
  std::vector<PvCandidate>      pv;        // bus, c_min, c_max, unit_size, capex, embodied_co2
  std::vector<EssCandidate>     ess;       // bus, p/e bounds, unit, capex_power, capex_energy, soh model
  std::vector<VscCandidate>     vsc;       // ac_bus, dc_bus, rating bounds, capex
  std::vector<SopCandidate>     sop;       // line, rating, capex            (flexible interconnection)
  std::vector<LineCandidate>    lines;     // from,to, build|upgrade, capex
  std::vector<SharedEssCandidate> shared_ess;
};

struct PlanningCase {
  HybridPowerSystem        base_system;    // the "do-nothing" network
  InvestmentCatalog        catalog;        // what may be built
  TimeSeriesData           profiles;       // 8760-h load/renewable
  CarbonTarget             carbon;         // baseline, reduction pathway λ→η, emission factors
  PlanningEconomics        econ;           // discount rate, horizon years, VOLL, $/tCO2
  ScenarioPolicy           scenario_policy;// #typical days, N-1 scope, resilience events
};

struct InvestmentDecision {                // first-stage solution = a buildable plan
  std::map<int,double> pv_mw, wind_mw, ess_mw, ess_mwh, vsc_mva;
  std::vector<int>     built_lines, built_sops, microgrid_assignment;
  double annualized_capex{0.0}, embodied_co2{0.0};
  HybridPowerSystem    as_built() const;   // materialize plan into a HybridPowerSystem for Fidelity-B
};

struct PlanningResult {
  std::vector<InvestmentDecision> pareto;  // over ρ
  InvestmentDecision   recommended;
  VerificationReport   verification;       // Fidelity-B metrics for `recommended`
  ConvergenceLog       log;                // UB/LB, #scenarios added, surrogate error
};
}
```

### 4.2 Key reuse points (no reinvention)

- **Investment model assembly** → `hacdcpf::aml` (= `mipsolvers::aml`); add binary/continuous investment vars and big-M coupling onto the LP emitted by `lindistflow_builder`.
- **Decomposition** → `engine/branch_and_cut` solves the master; subproblems are LPs solved by HiGHS via AML. Benders cuts assembled from subproblem duals.
- **Scenario engine** → `generate_scenarios` + a small clustering helper (k-medoids on `TimeSeriesData`).
- **Verification** → call the existing `solve_annual_production_simulation`, `run_*_reliability`, `run_distribution_resilience_*`, `run_carbon_analysis` unchanged; only marshal `InvestmentDecision::as_built()` in and metrics out.
- **Result attribution** → reuse `unproject_*` so plan results map back to rich components.

---

## 5. Decomposition & solver strategy

- **Primary:** multicut L-shaped Benders. Master = investment MILP (`branch_and_cut`); one optimality cut per scenario per iteration from LP recourse duals (HiGHS). Parallelize subproblems across scenarios (the engine already runs multithreaded eval).
- **Robust / refinement:** Column-and-Constraint Generation (CCG) for the worst-case resilience scenarios discovered by Fidelity-B — the adversary returns a new `ω` and we add its recourse block (this is the scenario-refinement step of the closed loop, expressed as CCG).
- **Monolithic fallback:** for small cases, solve the extensive-form MILP directly via `branch_and_cut` / Gurobi for validation of the decomposition.
- **Surrogate-embedded:** when Fidelity-C is active, only the **proposal master `M_C`** gains the ReLU big-M constraints; the **bound master `M_L`** stays pure relaxation. LB is read solely from `M_L`'s dual bound (theory §4.1).

---

## 6. Carbon & reliability coupling (the project's identity)

- **Carbon accounting** uses `run_carbon_analysis` per dispatch snapshot for *attribution* (which generator/import causes which load's CO₂), and an aggregate emission-factor term inside the MILP for *constraint enforcement*. Embodied carbon of equipment enters the first stage (`embodied_co2`). This matches the report's `Q_ds = ΣQ_g + Q_network + Q_ess + ρ_d ΣQ_d + ρ_g ΣQ_g^ext`.
- **Reliability/resilience** enter both as (a) a cheap proxy inside the MILP (priority load-shedding cost over `Ω_reliability`) and (b) the authoritative Fidelity-B indices (`EENS`, `SAIDI`, resilience index) that gate acceptance in the closed loop.
- **The coupling knob** is `ρ` (or carbon target `η`): sweeping it produces the cost↔carbon↔reliability Pareto front the report's 5-dimension evaluation consumes.

---

## 7. Implementation roadmap

### Phase 0 — Scaffolding & contracts (1 module, no math yet)
- Create `include/hacdcpf/planning/` + `src/planning/`, wire into `CMakeLists.txt`.
- Define `PlanningCase`, `InvestmentCatalog`, `InvestmentDecision`, `PlanningResult`.
- Implement `InvestmentDecision::as_built()` (materialize a plan into a `HybridPowerSystem`) and round-trip test it through `validate_full` + one `solve_power_flow`.
- **Exit test:** a hand-written plan builds, validates, and runs a power flow.

### Phase 1 — Deterministic single-scenario expansion (MVP)
- Extend `lindistflow_builder` with PV + ESS investment vars and big-M coupling.
- Solve a single-scenario investment MILP via `branch_and_cut`/HiGHS.
- **Exit test:** on a small radial feeder, planner sites/sizes PV+ESS to meet load at min cost; compare against Julia prototype on the same case.

### Phase 2 — Two-stage stochastic + Benders
- `scenario_set` (typical-day clustering) + multicut Benders (`benders.cpp`).
- Carbon constraint + VOLL load-shedding recourse.
- **Exit test:** UB/LB convergence on 8–14 typical days; matches extensive-form MILP within MIP gap.

### Phase 3 — Fidelity-B verification & closed loop
- `evaluator.cpp` runs annual/reliability/resilience/carbon on `as_built()`.
- `planning_loop.cpp` implements scenario refinement (CCG) when high-fidelity finds violations.
- **Exit test:** a plan accepted at Fidelity A but failing AC voltage / EENS at Fidelity B triggers a new scenario/cut and re-solve; loop converges to a plan that passes both.

### Phase 4 — Surrogate (Fidelity C) & multi-objective
- Sample → simulate → train surrogate; NN-as-MILP encoding (port `NeuralNetworkMILP.jl`).
- ρ-sweep → Pareto set; export to the report's 5-dimension evaluation dashboard.
- **Exit test:** surrogate-guided search reaches a comparable plan with ≥5× fewer Fidelity-B calls.

### Phase 5 — Multi-voltage / networked-microgrid extensions
- Add SOP / flexible interconnection, line build/upgrade, microgrid partitioning, shared/mobile ESS.
- Multi-year horizon via `run_lifecycle_simulation` in the evaluator.

---

## 8. Key design decisions (need a call before Phase 1)

1. **Implementation locus.** *Recommended:* implement the planner **natively in C++** inside this repo (reuses `aml`, `branch_and_cut`, all simulators in-process — no IPC, one data model). Alternative: keep optimization in Julia and call `hacdcpf` as a backend (faster to prototype, but two data models + serialization cost on every closed-loop iteration). The closed loop calls simulators *many* times, which argues strongly for in-process C++.
2. **MVP fidelity for the embedded recourse.** *Recommended:* LinDistFlow (radial AC, gives voltage) over pure DC — distribution voltage matters for the low-carbon DG hosting story.
3. **First investment elements.** *Recommended:* PV + ESS only in Phase 1 (highest value, simplest coupling), defer SOP/lines/microgrids to Phase 5.
4. **Solver dependency for the master MILP.** Native `branch_and_cut` (no license) vs. Gurobi (faster, licensed). *Recommended:* native first, Gurobi as optional accelerator behind the existing fallback chain.

---

## 9. Risks & mitigations

| Risk | Mitigation |
|---|---|
| Closed loop too slow (sim-in-loop expensive) | Surrogate (Fidelity C); cluster scenarios; parallel subproblems; escalate fidelity only on candidates that survive A. |
| Fidelity-A optimism never closes vs. Fidelity-B | CCG scenario refinement guarantees monotone addition of violated scenarios; bounded by finite N-1 set + sampled resilience. |
| MILP intractable at >2000 nodes (report scale) | Decomposition + network partitioning (reuse `LowCarbonDistributionSystemPlanning/network_partitioning/`); plan per partition then coordinate. |
| AC infeasibility of LinDistFlow-optimal plans | Phase-3 `solve_ac_opf` gate + feasibility cuts. |
| Two data models drift (if Julia kept) | Decision #1 → single C++ data model. |

---

## 10. Validation plan

- **Unit:** `as_built()` round-trips; investment coupling constraints; Benders cut correctness vs. extensive form on tiny cases.
- **Cross-check:** reproduce the Julia prototype's results on its test cases (Phase 1–2).
- **Fidelity gap study:** quantify how often Fidelity-A-optimal plans fail Fidelity-B (motivates the whole loop; becomes a paper figure).
- **Scale:** IEEE distribution test feeders → the report's 2000-node demonstration system.

---

## Appendix A — Mapping: report "7 planning modes" → this design

| Report mode | Realized by |
|---|---|
| 微电网规划 (microgrid planning) | Phase 5 partition + grid-forming assignment |
| 中压网架规划 (MV network) | Phase 5 line build/upgrade |
| 多电压协同 (multi-voltage coordinated) | SOP/VSC investment + hybrid AC/DC recourse |
| 源荷友好接入 (friendly DG/load access) | PV/wind/EV hosting in Phases 1–2 |
| 源荷网协同 (source-load-network) | full two-stage model, Phases 2–3 |
| 全周期仿真验证 (full-cycle sim verification) | Fidelity-B evaluator + `run_lifecycle_simulation` |
| 推演-决策闭环 (simulate-decide loop) | `planning_loop.cpp`, §3 closed loop |

## Appendix B — Reused `hacdcpf` API surface (no changes required)

`solve_power_flow`, `solve_ac_opf`, `solve_dc_opf`, `solve_unit_commitment`,
`solve_annual_production_simulation`, `run_lifecycle_simulation`,
`run_nonsequential_mc`, `run_distribution_fmea`, `run_three_stage_reliability`,
`run_distribution_resilience_mip_assessment`, `run_carbon_analysis`,
`generate_scenarios`, `validate_full`, `project_to_canonical_models`,
`hacdcpf::aml` (HiGHS/Gurobi/SCIP), `engine::branch_and_cut`,
builders: `lindistflow_builder`, `scuc_builder`, `dc_opf_builder`, `acopf_builder`.

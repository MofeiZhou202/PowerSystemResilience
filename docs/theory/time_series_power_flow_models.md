# Time-Series Power Flow — Mathematical Models, Algorithms, and In-Depth Analysis

> Documentation Sync (2026-07-12)
> Scope: derived directly from the implementation in `src/time_series/time_series_pf.cpp`
> and `include/hacdcpf/time_series/time_series_pf.hpp`.
> Status: implementation-backed reference with critical analysis.
> Source of truth: when text and implementation diverge, treat `src/`, `include/`,
> and `tests/` as authoritative.
>
> Companion documents: `sequential_production_simulation_rich_models.md`
> (per-component UC rich-model derivations §4.1–§4.9),
> `annual_simulation_models.md` (hierarchical annual / lifecycle layer built on
> top of this pipeline).

---

## 1. Problem statement and architecture

The time-series power flow (TSPF) engine answers the question: *given a hybrid
AC/DC distribution system and per-component scaling profiles over a horizon of
`T` steps of length `Δt` hours, what is a cost-(or carbon-)optimal, physically
consistent operating trajectory?* It is a **three-stage multi-fidelity
cascade**, each stage validating and refining the previous one:

```
 Stage 1 (scheduling)   Stage 2 (dispatch)         Stage 3 (verification)
 ┌──────────────────┐   ┌──────────────────────┐   ┌──────────────────────┐
 │ SCUC MILP        │──▶│ per-step AC-OPF      │──▶│ per-step hybrid      │
 │ DC power flow,   │   │ (parity IPM / Ipopt) │   │ AC/DC Newton PF      │
 │ inter-temporal   │   │ inside UC tracking   │   │ + cross-validation   │
 │ coupling         │   │ band                 │   │ metrics              │
 └──────────────────┘   └──────────────────────┘   └──────────────────────┘
    solve_unit_commitment      opf::solve_ac_opf        solve_handle (Newton)
```

Entry point: `solve_time_series_pf()` (`src/time_series/time_series_pf.cpp:slice_ts_window`).
Stage 1 alone: `solve_unit_commitment()` (`:3471`). The per-step system state is
materialised by `build_time_series_system_snapshot()` (`:3212`).

This is the same *schedule → dispatch → verify* chain used by commercial
production-costing tools (PLEXOS, PROMOD, PSO), but implemented here for
**hybrid AC/DC distribution networks** with explicit VSC / DC-DC converter
coupling — a combination not available in mainstream open-source tools
(pandapower's timeseries module has no UC; PyPSA has no AC-PF replay stage).

### 1.1 Sets and notation

| Symbol | Meaning | Code |
|---|---|---|
| $t \in \{0,\dots,T{-}1\}$, $\Delta t$ | time steps, step length (h) | `ts_data.num_steps`, `step_duration_hr` |
| $g \in \mathcal G$ ($G$ units) | in-service dispatchable generators | `gen_indices` |
| $s \in \mathcal S$, $d \in \mathcal S^{dc}$ | AC / DC storage (excl. hosting-capacity "static" ESS) | `storage_indices`, `dc_storage_indices` |
| $r \in \mathcal R$ | renewable generators | `renewable_indices` |
| $c \in \mathcal C$, $k \in \mathcal K$ | VSC / DC-DC converters | `vsc_indices`, `dcdc_indices` |
| $x \in \mathcal X$ | external grids (price-aware ties) | `ext_indices` |
| $b \in \mathcal B$, $n \in \mathcal B^{dc}$ | AC / DC buses | `net.n_bus`, `dcg.n_dc_bus` |
| $\pi_p(t)\ge 0$ | profile-$p$ scaling factor at step $t$ | `profile_value()` |

Profiles are dimensionless multipliers keyed by `profile_id` (convention:
profile 0 = system default load shape); external-grid *price* profiles are
absolute ¤/MWh series. Input validation (`:58`) rejects non-finite values,
duplicate ids, and non-positive `Δt`.

---

## 2. Stage 1 — Security-constrained unit commitment MILP

Built by `build_uc_milp()` (`:642`). The model is deliberately kept a **pure
MILP** (every term linear) so all four backends — HiGHS, SCIP, Gurobi, and the
in-tree branch-and-cut — can solve the identical matrix.

### 2.1 Decision variables (baseline blocks)

Per generator-step: dispatch $p_{g,t} \in [0, \bar P_g]$, commitment
$u_{g,t}\in\{0,1\}$, startup cost $s_{g,t}\in[0, c^{SU}_g]$.
Per storage-step (AC index $s$, DC index $d$, combined $k$): signed net power
$p^{ess}_{k,t}\in[\underline P_k,\bar P_k]$ (+ = discharge), state of charge
$e_{k,t}\in[\underline e_k,\bar e_k]$, **charging split** $p^{ch}_{k,t}\ge 0$,
and mode selector $z_{k,t}$. Per renewable-step: $p^{ren}_{r,t}\in[0,
\pi_r(t)\,P^{rated}_r]$. Per external grid: $p^{ext}_{x,t}\in[-\bar P^{ext},
\bar P^{ext}]$ (+ = import). With network constraints: bus angles
$\theta_{b,t}\in[-\pi,\pi]$ for non-slack buses; with the DC network:
converter powers $p^{vsc}_{c,t}$, $p^{dcdc}_{k,t}$ and optionally DC branch
transport flows $f_{m,t}\in[-\bar F_m,\bar F_m]$.

Opt-in blocks (demand response, PV curtailment claw-back, microgrid islanding,
storage degradation, VPP, energy router, mobile storage with or without
co-optimised relocation) each add their own variables; their derivations are
in `sequential_production_simulation_rich_models.md` §4. The complete variable
layout with offsets is documented at `src/time_series/time_series_pf.cpp:MobStor–1064`.

**Variable-count formula** (baseline, network + DC network on):

$$
n \;=\; \underbrace{3GT}_{p,u,s} + \underbrace{2(S{+}S^{dc})T}_{p^{ess},e}
+ \underbrace{2(S{+}S^{dc})T}_{p^{ch},z} + RT + XT
+ \underbrace{(N_b{-}1)T}_{\theta} + (C{+}C^{dc})T
$$

with $GT$ binaries plus $(S{+}S^{dc})T$ *conditional* binaries (see §2.4).

### 2.2 Objective — configurable, always linear

The objective mode (`UCObjective`) resolves to weights
$(w_{cost}, w_{CO_2}, w_{loss}, w_{curt})$ (`:1149`):

| Mode | $w_{cost}$ | $w_{CO_2}$ | $w_{loss}$ | $w_{curt}$ |
|---|---|---|---|---|
| Cost (default) | 1 | 0 | 0 | 0 |
| Carbon | 0 | 1 | 0 | 1 |
| MinCurtailment | $10^{-3}$ (tie-break) | 0 | 0 | 1 |
| MinLoss (proxy) | 0 | 0 | 1 | 1 |
| Weighted | user | user | user | user |

$$
\min \sum_{t}\Bigl[
\sum_g \bigl(\alpha_g\, p_{g,t} + w_{cost}\,c^0_g\, u_{g,t}\bigr)\Delta t + w_{cost}\, s_{g,t}
\;+\; \sum_r \bigl(w_{cost}\,c^1_r - \rho_r\bigr) p^{ren}_{r,t}\Delta t
\;+\; \sum_x \lambda_x(t)\, p^{ext}_{x,t}\Delta t
\;+\; \text{(opt-in penalty terms)}
\Bigr]
$$

where $\alpha_g = w_{cost}c^1_g + w_{CO_2}\,e_g + w_{loss}$ blends fuel price,
emission factor ($e_g$, tCO₂/MWh) and a generation-minimisation loss proxy;
the renewable *reward* $\rho_r = w_{cost}\,c^{curt}_r + w_{curt}$ makes
curtailment costly; $\lambda_x(t)$ is the profile-driven exchange price plus
$w_{CO_2}$ times the import emission factor.

Two design decisions deserve emphasis:

1. **Objective ≠ report.** Must-take sources (DC static gens, PV) are not
   decision variables, so their cost/emissions enter as a constant
   `obj_offset` (`:639`, `:1167–1206`) that is added back to the solver
   objective. Penalty encodings of the form $pen\cdot(1-o)$ are stored as
   $-pen\cdot o$ with the dropped constant tracked in the same offset, so the
   *reported* cost is always the true penalty form. Independently, the final
   monetary figure `total_generation_cost` is **recomputed from the resulting
   dispatch** (`:4412–4497`) regardless of which objective was optimised —
   a clean separation of the optimisation surrogate from the accounting metric.
2. **`MinLoss` is honest about its own limitation**: the UC DC power flow is
   lossless, so minimising loss is implemented as minimising total generation
   (a valid proxy only because demand is fixed net of DR), and the header
   documents this explicitly.

### 2.3 Network model — three fidelity tiers

**(a) Copper plate** (`enable_network_constraints=false`, `:1927`): one balance
row per step,

$$
\sum_g p_{g,t} + \sum_k p^{ess}_{k,t} + \sum_r p^{ren}_{r,t} + \sum_x p^{ext}_{x,t}
+ \dots = D_t,
$$

where $D_t$ is total profile-scaled load **net of must-take generation** (AC PV
via the physical PV curve `compute_pv_power_mw` when electrical parameters are
present, else linear scaling; DC PV/static gens folded in when the DC network
is not modelled, `:1791–1847`).

**(b) DC power flow on the AC grid** (`:1986`): per-bus balance

$$
\textstyle\sum_{\text{inj at } b} (\cdot)
\;-\; S_{base}\sum_{j} B_{bj}\,\theta_{j,t} \;=\; D_{b,t},
\qquad
\bigl|\tfrac{S_{base}}{x_\ell}(\theta_{f(\ell),t}-\theta_{t(\ell),t})\bigr| \le \bar F_\ell,
$$

with $B$ assembled from branch reactances and 2-winding transformers
(short-circuit voltage $x \approx \sqrt{v_k^2 - v_{kr}^2}/100$ rebased to
system MVA, `:499–520`). Any AC/DC residual (must-take DC sources without an
explicit DC grid) is assigned to the slack bus so the nodal model stays
balanced (`:1896–1921`).

**(c) Explicit DC network** (`enable_dc_network_constraints`, `:2196`): a DC
nodal balance per DC bus with **flat-voltage transport coupling**:

$$
\sum_{c \in \mathcal C(n)} \Bigl(-\tfrac{1}{\eta_c}\Bigr) p^{vsc}_{c,t}
+ \sum_{k:\, n = \text{out}(k)} \eta_k\, p^{dcdc}_{k,t}
- \sum_{k:\, n = \text{in}(k)} p^{dcdc}_{k,t}
+ \sum_{d \in \mathcal S^{dc}(n)} p^{dcess}_{d,t}
+ \sum_{m} A_{nm} f_{m,t}
= D^{dc}_{n,t},
$$

where $A_{nm} \in \{-1,+1\}$ is the branch incidence of the optional transport
flows ($f_{m,t}$ bounded by the branch rating **as a variable bound — zero
extra constraint rows**, `:1440–1451`). Without `enable_dc_branch_flows`, DC
power moves only through per-bus converter injections, and converter power
bounds act as the effective link capacities (documented design decision,
`:2648–2665`).

### 2.4 Storage — exact asymmetric-efficiency split with *conditional* binaries

This is the most theoretically interesting block (`:1288–1316`, `:2299–2357`,
`:2543–2570`). With signed net power $p = p^{dis} - p^{ch}$, only $p$ and
$p^{ch}\ge0$ are variables; the SOC row is

$$
e_{k,t} = \rho_k\, e_{k,t-1}
\;-\; \underbrace{\tfrac{\Delta t}{\eta^d_k E_k}}_{\text{coeff}_p}\, p_{k,t}
\;-\; \underbrace{\tfrac{\Delta t}{E_k}\bigl(\tfrac{1}{\eta^d_k}-\eta^c_k\bigr)}_{\text{coeff}_{ch}}\, p^{ch}_{k,t},
\qquad e_{k,-1} := e^{init}_k,
$$

with self-discharge retention $\rho_k = 1 - \sigma_k/100$. Substituting
$p = p^{dis} - p^{ch}$ recovers **exactly** the standard asymmetric model

$$
e_{k,t} = \rho_k\, e_{k,t-1} - \frac{p^{dis}_{k,t}\,\Delta t}{\eta^d_k E_k}
+ \frac{\eta^c_k\, p^{ch}_{k,t}\,\Delta t}{E_k}.
$$

The mode constraints are $p + p^{ch} \ge 0$ (i.e. $p^{dis}\ge0$),
$p + p^{ch} \le \bar P^{dis} z$, $p^{ch} \le \bar P^{ch}(1-z)$.

**Rigor result implemented in code:** the selector $z$ is declared binary
*only when the unit is bidirectional and lossy*
($\bar P^{ch},\bar P^{dis} > 0$ and $\eta^c\eta^d \ne 1$); otherwise it is a
continuous $[0,1]$ variable (`:1301–1315`). The justification: with unity
efficiencies, $\text{coeff}_{ch}=0$, so the SOC depends only on $p$ and any
fictitious simultaneous charge/discharge is SOC-neutral — the LP relaxation is
exact. With losses, $\text{coeff}_{ch} > 0$ means simultaneous
charge/discharge *destroys* energy; an LP could exploit that as free
curtailment under oversupply, so the binary is genuinely needed there. This
"binaries only where relaxation fails" policy minimises the B&B tree without
sacrificing exactness — a detail many published UC formulations get wrong by
always adding the binary (wasteful) or always omitting it (unsound under
negative prices / must-take surplus).

### 2.5 Unit-commitment logic constraints

Standard, all per generator (`:2475–2613`):

- **Capacity gating:** $p_{g,t} \le \bar P_g u_{g,t}$, $p_{g,t}\ge \underline P_g u_{g,t}$.
- **Ramping** (skipped when no limit): $|p_{g,t} - p_{g,t-1}| \le 60\,r_g\,\Delta t$.
- **Startup cost epigraph:** $s_{g,t} \ge c^{SU}_g (u_{g,t} - u_{g,t-1})$,
  $s_{g,t}\ge 0$, with $u_{g,-1} = \mathbb 1[p^{init}_g > 0]$ — exact under
  minimisation since $s$ has non-negative objective weight.
- **Min-up/down (pairwise form):** for $U_g = \lceil T^{up}_g/\Delta t\rceil \ge 2$:
  $u_{g,t} - u_{g,t-1} \le u_{g,t+k}$, $k = 1..\min(U_g{-}1,\,T{-}1{-}t)$, and
  symmetrically $u_{g,t-1} - u_{g,t} + u_{g,t+k} \le 1$ for min-down. The row
  count is data-dependent and — as the comment at `:1656–1674` records from a
  past defect — the counting loop must match the assembly loop *including the
  2-period case*, otherwise rows are silently written out of bounds.
- **Spinning reserve** (optional): $\sum_g (\bar P_g u_{g,t} - p_{g,t}) \ge
  r^{res}\, D_t$.

### 2.6 MILP solution strategy

`create_milp_adapter()` (`:2849`) and `solve_unit_commitment()` (`:3471`):

1. **Warm start.** A priority-list heuristic (`:3053`) builds a feasible-ish
   incumbent in $O(GT\log G)$: renewables at full availability, merit-order
   commitment to cover residual demand, remaining units at $\underline P$ if
   short, storage held neutral, startup costs back-filled. Typically < 1 ms;
   gives branch-and-cut an immediate upper bound.
2. **Backend selection.** `Auto` prefers HiGHS, falling back to the in-tree
   branch-and-cut tuned for UC: Gomory cuts (15 root rounds × 20 cuts),
   pseudocost branching, hybrid node selection, simplex LP nodes,
   **0.1 % gap tolerance** (production-costing appropriate — cost differences
   below 0.1 % are far inside data uncertainty), 20 000 LP iteration cap
   sized "for IEEE118+ root LPs" (`:2852–2874`).
3. **Thread control.** A thread-local override (`ScopedUCSolverThreadOverride`,
   `:2837`) lets the day-parallel decomposition cap each in-process B&C at one
   worker so day-level parallelism doesn't oversubscribe cores.

The solved vector is mapped back by `extract_schedule()` (`:2910`), including
dispatched-PV reconstruction as $\text{avail} - \text{curtailment}$ and
back-calculation of the trailing VSC/DC-DC blocks from the tail of $x$.

---

## 3. Stage 2 — Per-step AC-OPF inside a UC tracking band

For each step, `build_time_series_system_snapshot()` (`:3212`) constructs the
full nonlinear system state: generator dispatch/commitment from the UC,
storage power and SOC from the schedule (SOC also updates stored energy
$e = soc\cdot E$), profile-scaled loads / PV irradiance ($G = 1000\,\pi(t)$
W/m²), DC-side setpoints, and time-varying external-grid prices.

**Tracking band** (`:3295–3319`). For each online generator the OPF box is
tightened around the UC basepoint $p^{UC}$:

$$
\beta_g = \max\bigl(\beta^{abs},\ \beta^{rel}\max(|p^{UC}_g|, 1)\bigr),\qquad
[\underline P'_g, \bar P'_g] = \bigl[\max(\underline P_g,\, p^{UC}_g-\beta_g),\
\min(\bar P_g,\, p^{UC}_g+\beta_g)\bigr],
$$

with slack units at $(\beta^{rel},\beta^{abs}) = (20\%, 5\,\text{MW})$ and
non-slack units at $(2\%, 1\,\text{MW})$ by default; an empty interval
collapses to the clamped basepoint. Engineering interpretation: the OPF acts
as an **AGC-like re-dispatch layer** — the slack machine absorbs the losses
and DC-approximation error that the lossless UC could not see, while non-slack
units track their schedule so a single unit cannot silently take over the
system. Disabling the band (`enable_uc_opf_tracking_band=false`) turns Stage 2
into a free economic re-dispatch.

Storage is deliberately **not** re-dispatched by the OPF: replaying a
per-step OPF deviation would break consistency between power and the
schedule-derived SOC trajectory (comment at `:4247–4250`) — a subtle
inter-temporal-consistency argument that many pipelines overlook.

If the OPF diverges at a step, the pipeline degrades gracefully: PF runs on
the raw UC snapshot instead (`:4194–4204`).

---

## 4. Stage 3 — Hybrid AC/DC power-flow verification and cross-validation

The converged OPF's decisions are injected into the snapshot (generator P/Q,
VSC → PQ-mode setpoints, renewable dispatch via `ComponentRef` maps, DC-DC
output reference through the direction-aware conversion
`dcdc_output_power_from_input_ref_mw`, flexible-load realised demand, bus
voltage setpoints, and **proportional load shedding**: OPF shed $\Delta P_b$
is distributed across loads at bus $b$ pro-rata,
$P_i \leftarrow P_i - \Delta P_b\, P_i / \sum_j P_j$, `:342–416`).
A full Newton hybrid AC/DC PF (`solve_handle`) then verifies the state.

**Cross-validation metrics** per step (`CrossValStep`):

- $\max_b |V^{OPF}_{m,b} - V^{PF}_{m,b}|$ and
  $\max_b |(\theta^{OPF}_b - \theta^{PF}_b) - (\theta^{OPF}_0 - \theta^{PF}_0)|$
  — the angle comparison is **reference-aligned** via the first bus so
  different slack conventions don't produce spurious error (`:4308–4318`).
- Physical losses on both sides. PF losses = $\sum_\ell \max(P^f_\ell + P^t_\ell, 0)$
  + DC ohmic $\sum_m (\Delta V_m)^2 S_{base}/r_m$ + VSC and DC-DC converter
  losses. The OPF-side estimate (`estimate_opf_physical_loss_mw`, `:228`)
  recomputes branch flows from OPF voltages (after projecting authored bus
  vectors through the bus-merge map), evaluates the same converter model, and
  falls back to a boundary balance $|\Sigma\text{supply} - \Sigma\text{demand}|$
  if voltage vectors are unavailable.

These metrics are the pipeline's **trust certificate**: a small
$\max|\Delta V_m|$ means the OPF's network model was adequate; a large one
flags steps where the linearised schedule should not be trusted.

**Performance-relevant mechanics of the replay loop** (`:4113–4410`):

- One `SolverHandle` is created for the whole horizon and *reset* per step
  (`reset_solver_handle`) instead of rebuilt — preserving symbolic
  factorisation/pattern work across the $T$ Newton solves.
- A **topology cache** re-runs graph construction + island analysis only when
  some branch's `in_service` flag actually changed between steps (`:4116–4154`),
  reducing per-step overhead from $O(\text{graph build})$ to an $O(L)$ flag
  comparison in the common no-switching case.

---

## 5. Fallback scheduling when no UC schedule exists

When `skip_uc=true` (dynamic-OPF mode) or the MILP failed, storage still needs
an inter-temporal trajectory — a per-step OPF cannot invent one. The fallback
(`:3869–4099`) is a deterministic **peak-shaving rule with cyclic repair**:

1. Net load $\tilde D_t$ = profile-scaled load − renewables/PV; mean $\bar D$.
2. Each unit takes a share of the system signal proportional to its power
   rating: $p_{k,t} = \text{clip}\bigl((\tilde D_t - \bar D)\,
   \bar P_k / \Sigma_j \bar P_j,\ \underline P_k, \bar P_k\bigr)$,
   then clipped again so the exact asymmetric-efficiency SOC recursion stays
   inside $[\underline e, \bar e]$.
3. If terminal SOC must equal the initial SOC (cyclic), a greedy repair walks
   steps sorted by net load (cheapest hours first for charging, most expensive
   first for discharging), adjusting each step's power toward its headroom and
   finishing with a **60-iteration bisection** on the final adjustment so the
   terminal SOC lands on the target to $\sim 2^{-60}$ of the adjustment range
   while every intermediate SOC stays feasible (`:4025–4088`).

This is $O(T^2)$ per storage unit in the worst case (each candidate step
requires an $O(T)$ SOC re-simulation) — negligible at $T=24$, and still only
$\sim 10^8$ scalar ops per unit at $T=8760$. It is *not* optimal, but it is
feasible, deterministic, cyclic, and mirrors how a rule-based BESS controller
would behave — a defensible baseline when the optimiser is unavailable.

---

## 6. Per-day parallel decomposition (multi-day horizons)

`solve_time_series_pf` itself can split a multi-day horizon
(`parallel_daily=true`, `:3672–3737`): with $\tau = \text{round}(24/\Delta t)$
steps per day and $D = \lceil T/\tau\rceil$ days,

1. every day is solved as an **independent** TSPF with
   `enforce_terminal_soc_cyclic=true` — implemented as *variable-bound
   pinning* of the last SOC variable, $e_{k,\tau-1} = e^{init}_k$ (no extra
   rows, `:1257–1263`);
2. day results are stitched back by concatenating every per-step series
   (`concat_ts_results`, `:3607`).

**Why this is exact decomposition of the modified problem:** the only
constraints coupling adjacent days in the original MILP are the SOC recursion,
generator ramps, min-up/down windows, and startup states across midnight.
Pinning terminal SOC to the initial SOC makes the SOC chain block-diagonal;
the remaining cross-midnight couplings (ramp $t{=}0$, $u_{g,-1}$ from
`pg_mw`) are re-anchored to the static system state each day. The
decomposed problem is therefore a *restriction* of the coupled one
(feasible set shrinks: no inter-day energy arbitrage, no overnight
carry-over), so its optimal cost is an **upper bound** on the coupled optimum.
See `annual_simulation_models.md` §4.3 for the quantitative discussion.

**Thread-safety engineering** — a genuinely careful piece of systems work:

- The UC backend is re-resolved per day: `Auto → SCIP` if a MILP-capable SCIP
  adapter exists, else the in-tree B&C **capped at 1 thread per day**
  (`resolve_parallel_daily_uc_solver`, `:3184`). HiGHS and Gurobi hold
  process-global scheduler state, so explicit selections of those backends are
  *guarded to a serial day loop* rather than crashed or silently raced
  (`uc_solver_allows_parallel_daily`, `:3195`).
- Any per-step AC-OPF is forced onto the thread-safe native parity IPM
  (`ACOPFSolverBackend::ParityIPM`) so the non-reentrant Ipopt/MUMPS path is
  never entered concurrently (`:3684`).
- Workers write only index-addressed slots (`day_results[d]`) — lock-free by
  construction; provenance (`ParallelExecutionInfo`: mode, backend, guard
  reason, worker count) is reported to the caller for observability.

---

## 7. Component coverage and stage propagation — what is actually used where

The UC MILP *models* many component types, but modelling a component in
Stage 1 does **not** imply its optimised trajectory reaches Stages 2/3 or the
annual accounting. There are three independent cut points, each verified
against the code:

1. **Opt-in gating** — most rich blocks default off
   (`enable_dc_network_constraints`, `enable_vpp`, `enable_microgrid`,
   `enable_energy_router`, `enable_mobile_storage`, `enable_demand_response`,
   `enable_dispatchable_pv`, `enable_dc_branch_flows` are all `false` in
   `TimeSeriesPFOptions`). With defaults, the UC sees none of them.
2. **Schedule truncation** — `UCSchedule` has rows only for generators,
   AC/DC storage, renewables, AC PV/sgen, external grids, DC PV/sgen/load,
   VSC and DC-DC converters. `extract_schedule()` (`:2910`) therefore
   **discards** the optimised trajectories of VPPs, microgrids (exchange and
   islanding state), mobile storage (power, SOC, relocation), energy-router
   port flows, DR up/down, and DC branch flows — they exist only transiently
   in the MILP solution vector $x$.
3. **Snapshot non-application** — `build_time_series_system_snapshot()`
   (`:3212`) writes back generators, storage, renewables, loads, PV, DC-side
   setpoints, and external-grid prices. It **never** writes
   `vsc_converters[..].p_set_mw`, `dcdc_converters[..].p_ref_mw`,
   `vpps[..].p_output_mw`, `microgrids[..].p_exchange_mw` /
   `operating_mode`, or `mobile_storage[..].p_mw` — even for the VSC/DC-DC
   series that the schedule *does* carry.

### 7.1 Propagation matrix (verified)

★ = opt-in flag, default off. "authored" = the static value from the input
file, untouched by the UC.

| Component | Stage 1 UC | In `UCSchedule` | Applied to snapshot | Stage 2 OPF | Stage 3 PF |
|---|---|---|---|---|---|
| Generators | decision $(p,u,s)$ | ✅ | ✅ dispatch + commit + band | ✅ variable in band | ✅ |
| AC / DC storage | decision, exact $\eta$ split | ✅ $(p, e)$ | ✅ $p$, SOC→energy | pinned to schedule | ✅ |
| Renewable gens | decision $\le \pi P^{rated}$ | ✅ | ✅ | ✅ variable | ✅ |
| AC PV systems | must-take (★ dispatchable) | ✅ when ★ | ✅ | ✅ via `ren_map` | ✅ |
| DC PV / sgen / loads | profile replay (★ curtailable) | ✅ | ✅ | fixed injection | ✅ |
| External grids | ✅ (default on) | ✅ | price only (slack supplies power) | ✅ exchange variable | ✅ slack |
| **VSC converters** | ★ DC-network flag | ✅ extracted | ❌ **never applied** | **re-optimised** ($p^{ac},q^{ac}$) | native converter model |
| **DC-DC converters** | ★ DC-network flag | ✅ extracted | ❌ **never applied** | ✅ `pdcdc` → replayed to PF | native model |
| DC branch flows | ★ | ❌ discarded | — | real DC network | real DC network |
| **Flexible loads (DR)** | ★ decision | ❌ discarded | baseline `p_mw` (no profile scaling) | OPF's own flex variables | ✅ `pflex` replayed |
| **VPPs** | ★ decision | ❌ discarded | ❌ authored `p_output_mw` | **fixed injection** (`src/optimal_power_flow/parity_formulation.cpp:build_problem`) | fixed injection (`src/power_flow/solver_data.cpp:aggregate_generation`) |
| **Microgrids** | ★ exchange + islanding | ❌ discarded | ❌ authored `p_exchange_mw`, mode | **fixed injection** (`:548`) | fixed injection (`:77`) |
| **Mobile storage** | ★ (± co-relocation MILP) | ❌ discarded | ❌ authored `p_mw`/status | **fixed injection** (`:557`) | fixed injection (`:86`) |
| **Energy routers** | ★ port decision | ❌ discarded | ❌ | ✅ native port model (`src/optimal_power_flow/parity_formulation.cpp:build_problem`) | ✅ native Newton port model (`src/power_flow/jacobian_builder.cpp:build_power_spec`) |

Key readings of this table:

- **Nothing disappears from the physics.** Every component type is present in
  the Stage-3 PF: converters and energy routers through their native nonlinear
  models, VPP/microgrid/mobile-storage as fixed authored injections
  (`src/power_flow/solver_data.cpp:aggregate_generation–93`). What is lost is *optimised scheduling*, not
  presence.
- **VSC/DC-DC schedules are advisory.** The UC's converter series are
  extracted, concatenated, sliced by the annual layer, and serialised to JSON
  — but no consumer applies them to a solver state (verified by grep: the
  GUI's converter dispatch panels read *OPF* results). In the `run_opf=true`
  path this is defensible: the OPF re-derives converter flows on the exact AC
  model, and the UC series' real function is to shape commitment/storage
  decisions consistently with converter capacity. In the `run_opf=false` path
  (UC→PF direct, including the annual `DynamicSCED` mode) the PF's converter
  coordination solves converters from their *authored control modes* — the
  UC-optimal converter split is silently replaced by droop/setpoint logic.
- **VPP / microgrid / mobile-storage / ER / DR scheduling is
  fire-and-forget.** When enabled, their MILP blocks bend the generator and
  storage schedule (correctly), but the components themselves then operate at
  authored values in OPF/PF. The mismatch between "what UC assumed they do"
  and "what they do in replay" lands in the slack generator / external grid —
  and, if large, can push the per-step OPF against its tracking band and fail
  it. A UC that islanded a microgrid or relocated a mobile unit produces a
  replay in which neither event happens.
- **DR is doubly cut**: the UC's up/down decisions are discarded *and* the
  snapshot leaves flexible loads at unscaled baseline; only the OPF's own
  per-step flexibility model (independent of the UC's) reaches the PF.

### 7.2 Consequences and remediation path

For default-configured runs (all flags off) the pipeline is fully consistent —
the table's problem rows are exactly the opt-in blocks. The inconsistency is
therefore *opt-in* too: enabling `enable_vpp` et al. today buys
schedule-shaping, not trajectory replay. Closing the loop requires three
mechanical steps: (i) extend `UCSchedule` with rows for
VPP/MG/MS/ER/DR (and extract them — offsets are already recorded in
`UCBuildResult`), (ii) apply them in `build_time_series_system_snapshot`
(the storage pattern at `:3323` is the template, including the OPF
pinning-vs-schedule consistency argument), and (iii) count them in the annual
accounting perimeter (`annual_simulation_models.md` §7.2). Until then, results
for these components should be read from the UC objective/constraint effects,
not from the replay stages.

---

## 8. In-depth analysis

### 8.1 Engineering value

1. **A complete planning-grade operating study in one call.** The UC→OPF→PF
   cascade turns raw profiles into (a) a commitment/dispatch schedule with
   costs, (b) AC-feasible voltages/flows, and (c) a per-step trust metric.
   For distribution utilities evaluating PV/BESS/EV-charging integration on
   hybrid AC/DC feeders this replaces three separate tools and the error-prone
   glue between them.
2. **Converter-aware scheduling — with a scoped claim.** VSC/DC-DC coupling
   inside the UC MILP means DC-side storage and AC/DC exchange *capacity* are
   respected when committing units — essential for LVDC/MVDC feeders where
   the converter is the binding asset. DC storage schedules are replayed
   end-to-end; the converter power series themselves are advisory (re-derived
   by OPF/PF, §7.1), so the value is consistent commitment, not converter
   setpoint control.
3. **Objective plurality with accounting discipline.** Cost / carbon /
   curtailment / loss / weighted objectives share one constraint matrix, and
   the reported cost is always recomputed from physical dispatch. This lets a
   planner run "minimum-carbon vs minimum-cost" Pareto studies with strictly
   comparable accounting — a common source of silent error in ad-hoc studies.
4. **Operational realism knobs**: spinning reserve fraction, UC tracking bands
   (mirroring basepoint-following AGC), shiftable demand response, storage
   cycle-aging cost with daily cycle caps, price-aware grid exchange,
   microgrid islanding economics, mobile storage logistics. Each is opt-in
   with a documented default-off, so the baseline model stays auditable.
5. **Graceful degradation everywhere**: infeasible MILP → feasible=false
   schedule (not a crash); diverged OPF → PF on UC dispatch; missing UC →
   rule-based ESS fallback; missing profiles → unity scaling. For a GUI-driven
   tool (the web dashboard consumes `rich_results` per step) this robustness
   is what makes the feature usable by non-experts.

### 8.2 Theoretical rigor — strengths

- **Exact storage efficiency MILP** with a proof-backed minimal-binary policy
  (§2.4). The SOC recursion including self-discharge retention is the exact
  discrete-time model, not the common lossless approximation.
- **Restriction-based decomposability.** Cyclic terminal SOC as a variable
  bound is a mathematically clean way to make days independent: the
  decomposed optimum is a certified upper bound on the coupled optimum, and
  feasibility is preserved by construction.
- **Startup-cost epigraph and pairwise min-up/down** are standard, valid
  formulations (Carrión–Arroyo family). The min-up/down pairwise form is
  weaker (LP-relaxation-wise) than the extended turn-on/turn-off inequality
  systems, but with a 0.1 % gap target and warm start the practical impact is
  small at distribution scale.
- **Reference-aligned cross-validation** (angle offset removal) and
  projection-aware loss recomputation make Stage-3 metrics meaningful rather
  than convention-polluted.
- **Consistency guards**: the row-counting-vs-assembly mismatch class of bug is
  actively defended (`row != m_ineq` throws, `:2820`); TSD validation rejects
  NaN profiles up front.

### 8.3 Theoretical rigor — gaps and caveats (verified against code)

1. **Direction-asymmetric converter coupling is one-sided.** The DC balance
   uses a single coefficient $-p^{vsc}/\eta$ (`:2254–2259`). For DC→AC flow
   ($p^{vsc}>0$) this is the exact lossy model; for AC→DC ($p^{vsc}<0$) the DC
   bus is credited $|p^{vsc}|/\eta > |p^{vsc}|$, i.e. the linear model
   *creates* $\approx(1/\eta - \eta)|p| \sim 4\%$ energy at $\eta=0.98$
   instead of losing it. Same for reverse DC-DC flow ($-p$ at bus-in, $+\eta p$
   at bus-out). An exact bidirectional model needs an inflow/outflow split (as
   the energy-router block already does) or direction binaries. Stage 2/3
   correct the physics, but the *schedule* is biased toward AC→DC transfers.
   The code comment (`:2185–2194`) acknowledges the simplification but not the
   direction of the bias.
2. **Flat-voltage DC transport model.** Optional DC branch flows are a pure
   network-flow relaxation (no DC voltage variables, no ohmic loss, no
   Kirchhoff voltage law). Any transport-feasible flow is accepted even if no
   voltage profile realises it; conversely thermal limits are honoured. This
   is a *relaxation* on meshed DC grids (radial DC feeders are unaffected
   since flows are then uniquely determined by injections).
3. **Quadratic cost asymmetry.** The UC objective is linear ($c^1, c^0$,
   startup); $c^2 p^2$ is ignored in optimisation but *included* in the
   reported cost rate (`generator_operating_cost_rate` in the annual layer
   includes $c_2 p^2$). Schedules for systems with strongly convex costs are
   therefore mildly suboptimal relative to the reported metric, and the two
   figures are not perfectly commensurable. A piecewise-linear cost option
   would close this cleanly.
4. **`cv.opf_loss_mw = cv.pf_loss_mw` after PF convergence** (`:4354–4358`):
   the carefully computed OPF-side loss estimate is overwritten by the PF
   value for dashboard display, so `loss_diff_mw` is identically zero on the
   converged path. Defensible as a UI decision (one comparable loss metric),
   but it silently discards a genuine model-gap diagnostic; the independent
   estimate still exists at `:4289` for the non-overwritten variables.
5. **UC initial conditions are heuristic**: $u_{g,-1} = \mathbb 1[p_g^{init} >
   0]$ and no carry-in of prior min-up/down elapsed time. Fine for
   day-decomposed studies (each day re-anchors), but a rolling-horizon user
   should be aware midnight startup counting can be off by one unit-period.
6. **Reserve is capacity-only**: $\sum(\bar P u - p) \ge r D_t$ ignores ramp
   deliverability and storage headroom as reserve providers.
7. **Tracking band vs feasibility**: tightening non-slack units to ±max(2 %,
   1 MW) can render the per-step OPF infeasible when the lossless UC schedule
   is far from AC-feasible (heavy losses, voltage constraints). The pipeline
   handles it (falls back to PF-on-UC), but the failure is reported as
   OPF non-convergence rather than attributed to the band.

### 8.4 Computational performance

**Model size.** For the baseline network-constrained UC:
rows $m \approx (N_b + N_{dc})T + (S{+}S^{dc})T$ equalities and
$m_{ineq} \approx 5GT + 3(S{+}S^{dc})T + 2L^{rated}T + \Sigma_g(U_g{+}D_g)T$
inequalities; nonzeros are $O\bigl(T\,(\text{nnz}(B) + \text{components})\bigr)$.
Worked example — a 33-bus feeder, $G{=}3$, $S{=}2$, $R{=}2$, $T{=}24$:
$n \approx 1.1\text{k}$ variables, ~72 commitment binaries; the tuned B&C or
HiGHS solves this in the tens of milliseconds. An IEEE-118-class day
($G{\approx}54$, $T{=}24$) gives $n\approx 15\text{k}$, ~1.3k binaries — the
stated design point of the LP-iteration cap.

**Assembly cost is engineered, not incidental.** The nodal balance assembly
converts $B$ to row-major once so each bus row iterates only its structural
non-zeros: $O(T\cdot\text{nnz}(B))$ instead of the naive $O(T N_b^2)$ — the
comment records a measured 3–5× (`:2080–2084`). Constraint counting is
performed symbolically before assembly so triplet vectors and RHS are exactly
sized (no reallocation).

**Warm start + 0.1 % gap** dominate MILP wall time in practice: the
priority-list incumbent bounds the tree immediately, and UC objectives are
shallow near the optimum, so most day instances close at the root or within a
few nodes.

**Replay stage.** $T$ AC-OPF solves (interior-point, the expensive part) +
$T$ Newton PF solves with handle reuse and topology caching. Per-step cost is
$O(\text{IPM iterations} \times \text{KKT factorisation})$; for distribution
feeders the OPF dominates end-to-end runtime, which is why the annual layer's
`DynamicSCED` mode (no per-step OPF) exists.

**Parallel decomposition.** Day solves are embarrassingly parallel:
wall-clock $\approx \lceil D/W\rceil \cdot t_{day}$ with
$W = \min(\text{hardware threads}, D)$ workers, minus the serial stitch
($O(T)$ concatenation). Because a day MILP is solved in near-constant time and
MILP complexity is superlinear (worst-case exponential) in horizon length, the
decomposition converts an intractable coupled multi-day MILP into $D$
easy MILPs — the dominant speedup, with thread-parallelism a further
$\times W$ on top. The backend guard matrix (SCIP/native parallel;
HiGHS/Gurobi serial-guarded; `skip_uc` always parallel) trades peak throughput
for correctness on shared-state solvers, and reports which branch was taken.

**Memory.** Results are $O(T)$ vectors of per-step structs;
`keep_system_snapshots` (a full `HybridPowerSystem` copy per step) is the only
$O(T\cdot|\text{sys}|)$ item and is opt-in. `rich_results` reconstruction
(`:4499`) re-projects each step through the rich→canonical→rich operators —
$O(T\cdot|\text{components}|)$, done once after the solve.

### 8.5 Validation surface

Behavioural tests pin each opt-in block: `tests/test_uc_storage_efficiency.cpp`
(exact split), `test_uc_dc_branch_flows.cpp`, `test_uc_external_grid.cpp`,
`test_uc_demand_response.cpp`, `test_uc_dispatchable_pv.cpp`,
`test_uc_microgrid.cpp`, `test_uc_storage_degradation.cpp`, `test_uc_vpp.cpp`,
`test_uc_energy_router.cpp`, `test_uc_mobile_storage.cpp`; the pipeline end to
end in `tests/test_multiscale_comprehensive.cpp` (24-step daily → 365-block
annual chronology, energy-balance and ENS assertions).

---

## 9. Summary judgement

The TSPF engine is a production-costing pipeline with genuinely careful
mathematics where it matters most — exact storage efficiency with minimal
binaries, restriction-based day decomposition, objective/accounting
separation, reference-aligned cross-validation — wrapped in defensive systems
engineering (solver guards, topology caches, graceful degradation). Its main
formal weaknesses are the one-sided linear converter-loss coupling (bias
toward AC→DC transfers at schedule level), the flat-voltage DC transport
relaxation on meshed DC grids, the linear-vs-quadratic cost mismatch between
optimisation and reporting, and — most consequential for users of the rich
component set — the stage-propagation cuts of §7: VPP / microgrid / mobile
storage / energy-router / DR schedules are optimised but discarded, and even
the retained VSC/DC-DC series are advisory. The first three weaknesses are
visible in Stage-3 cross-validation; the propagation cuts are structural and
need the `UCSchedule` / snapshot / accounting extensions sketched in §7.2
before the opt-in component models can be considered end-to-end usable.

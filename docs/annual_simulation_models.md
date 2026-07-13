# Annual Production Simulation & Lifecycle Layer — Mathematical Models, Algorithms, and In-Depth Analysis

> Documentation Sync (2026-07-12)
> Scope: derived directly from `src/time_series/annual_production_sim.cpp`,
> `src/time_series/lifecycle_simulation.cpp` and the corresponding headers in
> `include/hacdcpf/time_series/`.
> Status: implementation-backed reference with critical analysis.
> Source of truth: when text and implementation diverge, treat `src/`,
> `include/`, and `tests/` as authoritative.
>
> Companion documents: `time_series_power_flow_models.md` (the UC→OPF→PF
> pipeline every sub-horizon is solved with),
> `sequential_production_simulation_rich_models.md` (per-component UC models).

---

## 1. Problem statement

Annual production simulation evaluates a hybrid AC/DC distribution system over
a full year ($T_{yr}$ steps, canonically 8760 hourly or 1460 six-hourly) and
reports energy, cost, emissions, curtailment, reliability (ENS), losses, and
component utilisation statistics. The naive formulation — one coupled
security-constrained UC MILP over 8760 steps — is computationally intractable
(binary count $G\cdot T_{yr}$; MILP worst case exponential) and numerically
pointless (input profiles carry far more uncertainty than the optimality gap
of any tractable relaxation). The module therefore offers **two decomposition
strategies** over the same per-horizon solver
(`solve_time_series_pf`, see companion doc):

1. **Hierarchical temporal decomposition** L0→L2→L3 (sequential), and
2. **Parallel daily decomposition** (365 independent, energy-neutral days).

On top sits a **multi-year lifecycle layer** (degradation, load growth,
replacement economics, carbon accounting with *a-priori* error bounds).

Entry points: `analysis::solve_annual_production_simulation()`
(`src/time_series/annual_production_sim.cpp:1278`),
`analysis::run_lifecycle_simulation()`
(`src/time_series/lifecycle_simulation.cpp:345`).

---

## 2. Hierarchical temporal decomposition (sequential path)

### 2.1 Level structure

$$
\underbrace{\text{L0: annual blocks}}_{\text{12 months / 52 weeks}}
\;\longrightarrow\;
\underbrace{\text{L2: weekly SCUC}}_{168\,\text{h bind} + 48\,\text{h lookahead}}
\;\longrightarrow\;
\underbrace{\text{L3: daily replay}}_{24\,\text{h, UC pinned, OPF+PF per step}}
$$

**L0 — annual plan** (`solve_annual_plan`, `:824`). Blocks are calendar months
(day-count exact, remainder absorbed by December) or 52 weeks
(`build_block_ranges`, `:44`). Each `AnnualPlanBlock` carries maintenance
flags, per-generator energy budgets, a fuel budget, and storage SOC boundary
targets. In the current implementation L0 is a **heuristic seed, not an
optimisation**: maintenance is empty, energy budgets default to the trivial
bound $\bar P_g \cdot |{\rm block}| \cdot \Delta t$, fuel budget $10^{30}$,
SOC boundaries equal to `soc_init`, and the block cost estimate is

$$
\hat J_b = \Bigl(\sum_{t\in b}\sum_{\ell} P_\ell\,\pi_\ell(t)\,\Delta t\Bigr)\cdot
\bar c_1, \qquad \bar c_1 = \tfrac{1}{|\mathcal G|}\sum_g c^1_g,
$$

i.e. block energy times the *unweighted average* marginal cost. The data
structures for a true L0 MILP (budgets, maintenance, SOC targets) exist and
flow downward, but nothing populates them non-trivially yet. Likewise
`iterative_feedback` / `max_feedback_iterations` /
`budget_violation_tol_mwh` are declared in `AnnualProductionSimOptions`
(header `:234–236`) **and never referenced by the orchestrator** — the
bottom-up feedback loop is an unimplemented interface. This is the single
largest gap between the documented architecture and the code.

**L2 — weekly rolling SCUC** (`solve_weekly_uc`, `:923`). For week $w$
starting at $t_w$, one UC MILP is solved on the window
$[t_w,\ \min(t_w + 168/\Delta t + L,\ T_{yr}))$ where
$L = 48/\Delta t$ lookahead steps. Only the binding 168 h are replayed; the
lookahead exists to give storage and min-up/down decisions terminal context so
end-of-week myopia (draining every battery at hour 167) is suppressed. Block
controls (maintenance outages, SOC init) are applied to the system copy first
(`apply_annual_block_controls`, `:900`).

**L3 — daily replay** (`solve_daily_replay`, `:954`). Each day slices the
weekly UC (`[uc_offset, uc_offset + \tau)` rows of every dispatch/commit/SOC
series) into a `precomputed_uc_schedule` and calls `solve_time_series_pf` with
`skip_uc=false` but the schedule pinned — the expensive MILP is skipped while
profile replay, per-step AC-OPF (inside the UC tracking band), PF validation,
and fallback completion still run. Weekly commitment thus remains binding
while intra-day physics is verified at full fidelity.

### 2.2 Inter-level coupling — what is and is not enforced

Downward: maintenance and SOC init (L0→L2/L3), commitment + dispatch bands
(L2→L3). Upward: nothing (see above). **Inter-week storage continuity is not
enforced**: every week restarts at the L0 SOC target, which defaults to
`soc_init`. The implicit model is week-cyclic storage. For daily/weekly
cycling batteries this is benign; for seasonal storage (large hydro,
hydrogen) it structurally suppresses the inter-week arbitrage that a coupled
model would find — the same restriction argument as the daily decomposition
(§4.3) at one level up.

---

## 3. Parallel daily decomposition (`enable_parallel_daily`)

`solve_parallel_daily` (`:1071`) bypasses L0–L3 entirely: the year is
partitioned into $D = \lceil T_{yr}/\tau \rceil$ calendar days
($\tau = $ `daily_window_hours`$/\Delta t$), each solved as an independent
`solve_time_series_pf` horizon with **per-day cyclic SOC**
($e_{k,\tau-1} = e^{init}_k$, imposed as a variable bound), then stitched.

### 3.1 The three daily modes

| Mode | UC MILP | Per-step OPF | Commitment | Problem class / day |
|---|---|---|---|---|
| `SCUC` | yes | yes | optimised (binary) | MILP + $\tau$ NLPs |
| `DynamicSCED` | yes (binaries pinned) | no | from a representative day, else all-ON | **LP** (+ storage-mode binaries if lossy) |
| `DynamicOPF` | no | yes | n/a (`skip_uc`) | $\tau$ NLPs + ESS fallback rule |

`DynamicSCED` commitment source (`:1095–1134`): when
`sced_reuse_scuc_commitment=true`, the **peak-load day** (max of profile 0
within each day) is solved once as a full SCUC and its binary schedule
$u^\ast_{g,t}$ is replayed on every other day via
`fixed_commitment_schedule` — variable bounds $u_{g,t} := u^\ast_{g,t}$
collapse the MILP to a multi-period economic dispatch with ramps and network
limits. Fallback: all in-service units pinned ON. This is the classic
"representative-day commitment" heuristic from production-cost practice, and
its bias is predictable: commitment sized for the peak day is feasible but
over-committed on low-load days (extra no-load cost, less startup cycling).

### 3.2 Concurrency model

Identical guard architecture to the TSPF layer: SCIP or single-thread native
B&C per day (parallel-safe), HiGHS/Gurobi guarded to a serial loop; OPF forced
to the thread-safe parity IPM; each worker writes only its own indexed slots
(`day_scheds[d]`, `day_snaps[d]`, `result.step_results[g0..g1)`) so the region
is lock-free except one `std::atomic<int>` feasibility counter. Wall-clock
$\approx \lceil D/W \rceil \cdot t_{day}$, $W = \min(\text{cores}, D)$;
provenance (`parallel-daily/{scip|native-bc|no-milp}` vs `serial-guarded`)
is reported.

---

## 4. Accounting mathematics

### 4.1 Per-step energy bookkeeping (`fill_step_results`, `:467`)

For each global step, with UC/OPF quantities where available and
profile-driven values as fallback (`reported_profile_dispatch`, `:221`):

$$
\text{supply}_t = P^{gen}_t + P^{ren}_t + P^{dis}_t + [P^{ext}_t]^+ ,\qquad
\text{demand}_t = P^{load}_t + [P^{loss}_t]^+ + P^{ch}_t + [-P^{ext}_t]^+,
$$

$$
\varepsilon_t = \text{supply}_t - \text{demand}_t \quad (\text{reported as
\texttt{power\_balance\_error\_mw}}).
$$

**Closure rule.** When the step's PF converged, losses become the *realised*
PF losses (AC branch $\sum(P^f{+}P^t)$ + DC ohmic + converter losses) and the
external-grid exchange is **re-derived as the residual**

$$
P^{ext}_t := \bigl(P^{load}_t + [P^{loss}_t]^+ + P^{ch}_t\bigr)
- \bigl(P^{gen,UC}_t + P^{ren}_t + P^{dis}_t\bigr)
$$

(`:568–593`). The code comment explains why: pairing the *scheduled* grid
exchange with *realised* PF losses mixes two operating points and produces a
visible annual balance gap. This closure makes the annual energy identity
exact by construction — at the cost that the reported grid exchange is a
derived quantity, not the OPF's decision (see §7.2).

Step cost rate (¤/h) is assembled from the OPF dispatch when converged, else
the UC dispatch, using each unit's full cost curve
$c_0 u + c_1 p + c_2 p^2$ (`generator_operating_cost_rate`, `:400`), plus
profile-priced must-take sources, storage bids, scheduled grid exchange
priced by $c_1^{x}\cdot\pi_{price}(t)$, and — when no exchange was scheduled —
an implicit import term $[\text{need}-\text{supply}]^+$ priced at the average
external tariff (default 20 ¤/MWh when unpriced).

**Accounting perimeter — which components are counted.** The supply/demand
terms above are built from `UCSchedule` rows plus profile-driven fallbacks
(`reported_profile_dispatch`, `:221`): generators, AC/DC storage, renewables,
AC/DC PV and static generators, external grids, AC/DC loads and charging
stations. **VPPs, microgrids, mobile storage, energy routers, and DR
deviations are outside this perimeter** — `UCSchedule` carries no rows for
them (see the propagation matrix in `time_series_power_flow_models.md` §7.1)
and `fill_step_results` never reads the system's `vpps` / `microgrids` /
`mobile_storage` / `energy_routers` tables. Their real injections *are* in
the PF solution, so on PF-converged steps the closure rule below silently
attributes their energy to the derived external-grid exchange; on
non-converged / schedule-only steps it surfaces as
`power_balance_error_mw`. An annual study whose fleet includes a large VPP
or a routinely-exporting microgrid will therefore report distorted
import/export series while still closing its energy balance.

### 4.2 Aggregation

Block summaries integrate power → energy, $E = \sum_t P_t\,\Delta t$
(`aggregate_block`, `:613`). Component statistics (`:644–818`):

- Generator: energy, capacity factor $CF_g = E_g / (\bar P_g \cdot 8760\,h)$,
  startups/shutdowns counted from commitment edges $u_{t-1}{=}0 \wedge u_t{=}1$,
  online hours.
- Storage: throughput split, equivalent cycles
  $N^{cyc} = E^{dis} / E^{rated}$ (full-depth-equivalent convention).
- Renewables: dispatched energy, curtailment
  $E^{curt}_r = \sum_t [\pi_r(t) P^{rated}_r - p^{ren}_{r,t}]^+ \Delta t$,
  curtailment rate $E^{curt}/(E^{disp}+E^{curt})$, capacity factor.

### 4.3 Decomposition error — formal characterisation

Let $J^\ast_{cpl}$ be the coupled-year optimum and $J^\ast_{dec}$ the sum of
day optima under cyclic SOC. Every day-decomposed feasible point is feasible
for the coupled problem (concatenate the days; SOC is continuous at midnight
because each day starts and ends at $e^{init}$), hence

$$
J^\ast_{cpl} \;\le\; J^\ast_{dec},
$$

with the gap equal to the value of inter-day flexibility: overnight/weekly
energy arbitrage, avoided startups across midnight, ramp smoothing at day
boundaries. For batteries with daily price/load cycles the gap is
empirically small (the optimal coupled trajectory is itself nearly
day-periodic); it grows with storage duration (energy/power ratio ≫ 24 h) and
with multi-day weather persistence. The same bound applies week-wise for the
hierarchical path. Cross-midnight ramp/min-up violations at the stitch are
possible in principle (each day re-anchors $u_{g,-1}$ from the static system
state) but are bounded by one ramp interval and are visible in the Stage-3 PF
if they matter physically.

---

## 5. Lifecycle simulation (multi-year layer)

`run_lifecycle_simulation` (`lifecycle_simulation.cpp:345`) iterates years
$y = 1..Y$ over a mutable working system.

### 5.1 State evolution

$$
\text{load: } P^{load}(y) = P^{load}_0 (1+g)^{y-1},\qquad
\text{PV: } \bar P^{pv}(y) = \bar P^{pv}_0 (1-\delta)^{y-1},
$$

$$
SOH^{cal}(y) = 1 - a\,(y{-}1),\quad
SOH^{cyc}(y) = 1 - b\,N^{cyc}_{cum},\quad
SOH = \min(SOH^{cal}, SOH^{cyc}),
$$

with effective capacity $E^{rated}(y) = E^{rated}_0 \cdot SOH$. Replacement
triggers when $SOH \le SOH^{EOL}$: cost
$c^{repl}\cdot E^{rated}_0 \cdot 1000$ is booked, SOH and cycle counters
reset. Cycle counts accumulate from the annual simulation's storage stats.
Defaults: $g{=}2\%$, $\delta{=}0.5\%$, $a{=}2\%/\text{yr}$,
$b{=}4{\times}10^{-5}/\text{cycle}$ (25 000 cycles → 100 %).

Economics: net present value with discount rate $r$,

$$
NPV = \sum_{y=1}^{Y} \bigl(J_y + C^{repl}_y\bigr)(1+r)^{-(y-1)}.
$$

Each year runs a **Tier-1** annual simulation in schedule-only mode
(`skip_replay=true`, `run_opf=false`) — weekly UC only, no OPF/PF — the speed
tier that makes $Y{=}20$ years × parameter sweeps tractable.

Note that the lifecycle loop constructs its `AnnualProductionSimOptions`
**fresh with defaults** (`lifecycle_simulation.cpp:517–521`): every opt-in UC
block (`enable_network_constraints`, `enable_dc_network_constraints`,
`enable_vpp`, `enable_microgrid`, `enable_energy_router`,
`enable_mobile_storage`, `enable_demand_response`, …) is therefore **off** in
lifecycle studies regardless of how the caller configured other layers. A
lifecycle system containing VPPs, microgrids, mobile storage, or energy
routers is evaluated with those assets frozen at authored setpoints — and
since `skip_replay=true` also skips the PF stage, they contribute *nothing*
to the yearly energy/cost/carbon figures. Only generators, storage,
renewables, PV/sgens, external grids, and loads drive lifecycle economics
today.

### 5.2 Tiered carbon estimation with a-priori error bounds

The layer's distinguishing feature is that every yearly carbon number ships
with a **decomposed theoretical error bound**
$\varepsilon = \varepsilon_{disp} + \varepsilon_{samp} + \varepsilon_{stor}$:

**(a) Dispatch (loss-proxy) bound** (`compute_dispatch_bound`, `:95`).
Tier 1 is a lossless dispatch; the unmodelled network loss corrupts carbon by
at most

$$
\varepsilon_{disp} \le \bar e \cdot L_{max} \sum_t P^{load}_t \Delta t,
$$

with $\bar e$ the max emission factor and $L_{max}$ the loss fraction proxy
(default 3 %). This is a valid worst-case: every lost MWh is re-generated by
the dirtiest unit.

**(b) Stratified-sampling bound** (`compute_sampling_bound`, `:111`). Hours
are classified into 7 operating strata (`classify_strata`, `:20`): peak load
(≥ 0.85 max), low renewable (≤ 0.1 max), high curtailment, heavy charge /
discharge (≥ 0.5 of extremes), low load (≤ 0.3 max), normal. Tier 2 samples
$n_m$ hours per stratum for detailed-PF correction, allocated by **Neyman
allocation**

$$
n_m = n_{tot}\,\frac{N_m \sigma_m}{\sum_j N_j \sigma_j},
$$

where the correction std-dev is modelled as
$\sigma_m = L_{max}\,\bar e\,\sigma^{gen}_m$ (the PF correction scales with
loss fraction × emission factor × within-stratum generation spread). The
estimator variance uses the standard stratified form **with finite-population
correction**:

$$
\mathrm{Var} = \sum_m \frac{N_m^2}{n_m}\,\sigma_m^2\Bigl(1-\frac{n_m}{N_m}\Bigr),
\qquad
\varepsilon_{samp} = z_{1-\alpha/2}\sqrt{\mathrm{Var}},
$$

$z \in \{1.28, 1.645, 1.96, 2.576\}$ by confidence level. Correctly, the
bound covers **only the PF-correction residual**, not absolute carbon — Tier 1
already provides all-hours dispatch, so the dense sum
$\hat W = \bar e\,\Delta t \sum_t P^{gen}_t$ (`compute_dense_carbon`, `:291`)
is the primary estimator and sampling only refines it.

**(c) Storage carbon propagation bound**
(`compute_storage_carbon_bound`, `:227`). Charging-hour carbon intensity
propagates into discharge accounting; between sampled hours the intensity
$w(t)$ is interpolated, giving the Lipschitz bound

$$
|\hat w - w| \le \tfrac{1}{2} K_w\, \Delta_{max},\qquad
\varepsilon_{stor} = \tfrac{1}{2} K_w \Delta_{max} \cdot
\Bigl(\sum_t |P^{ess}_t|\Delta t\Bigr)\cdot \max(1, N_{stor}),
$$

with $\Delta_{max}$ the largest gap (hours) between sampled hours and $K_w$
estimated from the max step-to-step generation change **scaled by a hardcoded
0.001 tCO₂/MWh per MW factor** (`:241`).

**(d) Cross-validation** (`BoundCrossValidation`): the dense and
sampled-extrapolated estimates are compared
($\text{gap} = |\hat W_{dense} - \hat W_{samp}|$, reported in % of dense) and
each bound component is expressed as % of dense carbon — an empirical
tightness check of the theoretical bounds, per year.

### 5.3 Capacity comparison sweeps

`run_lifecycle_comparison` (`:722`) sweeps one scale factor
(pv / wind / bess_power / bess_energy / diesel) across values, cloning the
system, applying `apply_capacity_scaling` (`:652`, sign-preserving on
`pmin_mw`, SOC reset to 50 %), and re-running the full lifecycle per scenario
— the computational substrate for NPV-vs-carbon planning frontiers.

---

## 6. Computational performance analysis

**Complexity budget.** Let $t_{UC}(\tau)$ be the day/week MILP time,
$t_{OPF}$, $t_{PF}$ per-step solve times.

| Path | Cost model | Notes |
|---|---|---|
| Hierarchical | $\sum_{w=1}^{52} t_{UC}(168{+}48) + T_{yr}(t_{OPF}+t_{PF})$ | serial; weekly MILP (168·G binaries) is the pacing item |
| Parallel SCUC | $\lceil 365/W\rceil\,[\,t_{UC}(24) + \tau(t_{OPF}{+}t_{PF})]$ | near-linear speedup to $W=\min(\text{cores},365)$ |
| Parallel DynamicSCED | $t_{UC}^{SCUC}(24) + \lceil 365/W\rceil\, t_{LP}(24)$ | one MILP total; days are LPs |
| Parallel DynamicOPF | $\lceil 365/W\rceil\,\tau\, t_{OPF}$ | no MILP; ESS by fallback rule |
| Lifecycle (per scenario) | $Y \times$ hierarchical-with-`skip_replay` | weekly UC only, no OPF/PF |

The decisive effect is **structural, not just parallel**: a coupled 8760-step
SCUC has $8760G$ binaries and is out of reach for any backend here, while 365
24-step SCUCs each carry $24G$ binaries and solve in near-constant time —
day decomposition converts superlinear (worst-case exponential) growth in $T$
into linear growth, and threads then divide the constant. `DynamicSCED`
removes even that: one representative MILP, then LPs.

**Memory.** `step_results` is $O(T_{yr})$ small structs; PF snapshots are
sampled every `pf_snapshot_interval` steps (default 24 → 365 snapshots/yr,
each $O(N_b + L)$) rather than stored densely — the dashboard gets voltage/flow
detail at bounded memory. Weekly schedules retain full UC series
($O(52\cdot(G{+}S{+}R)\cdot 216)$ doubles) for the statistics pass.

**Engineering details that matter at scale**: profile slicing copies only the
window (`slice_ts_data` pads with 1.0 past the end); per-day outputs are
index-addressed (no locks, no false sharing on hot paths); monthly aggregation
and component statistics reuse the same helpers on both paths, so parallel and
sequential results are directly comparable.

---

## 7. In-depth analysis

### 7.1 Engineering value

1. **The right tool per question, one data model.** SCUC days for operational
   fidelity, DynamicSCED for fast annual production costing, DynamicOPF for
   network-limit screening, `skip_replay` for lifecycle sweeps — all driven by
   the same system/profile inputs and returning the same result schema, so a
   planner can escalate fidelity only where a cheap pass shows stress.
2. **Planning-grade outputs**: monthly energy balances that close by
   construction, per-unit capacity factors and startup counts (maintenance and
   cycling wear inputs), storage equivalent cycles (degradation input),
   curtailment rates (interconnection studies), ENS (reliability), and PF
   snapshots for dashboard visualisation — the exact deliverable set of a
   commercial production-cost study, natively for hybrid AC/DC feeders.
3. **Carbon numbers with uncertainty statements.** Reporting
   $\hat W \pm \varepsilon$ with a decomposed, auditable bound (dispatch proxy
   / sampling / storage propagation) is substantially more defensible in
   regulatory or academic use than the point estimates typical of planning
   tools, and the per-year cross-validation exposes when the bounds are loose.
4. **Lifecycle economics closed-loop**: degradation feeds capacity feeds
   dispatch feeds cycles feeds degradation — with replacement events and NPV,
   enabling honest BESS sizing trade-offs (`run_lifecycle_comparison`
   frontiers) rather than static-capacity assumptions.

### 7.2 Theoretical rigor — strengths and verified caveats

**Strengths.**

- The decomposition is a *restriction* with a one-sided optimality guarantee
  (§4.3) — mathematically clean, feasibility-preserving, and honestly
  reported (`solver_name` records mode and parallel/serial branch).
- Stratified sampling is done properly: Neyman allocation, finite-population
  correction, bounds scoped to the correction residual only (not the whole
  estimate), explicit z-quantiles, per-stratum bookkeeping in the result.
- The energy-closure rule eliminates the OPF-loss/UC-exchange mixing artefact
  and makes the annual identity $\sum(\text{supply}-\text{demand})\Delta t
  \equiv \text{balance error}$ meaningful.
- Representative-day SCED commitment has a known, one-signed bias
  (over-commitment on off-peak days → conservative cost, feasible schedules).

**Caveats (all verified against the code).**

1. **L0 is a stub and the feedback loop is unimplemented.** The plan is
   heuristic (average-marginal-cost block costs, unconstrained budgets), and
   `iterative_feedback`/`max_feedback_iterations`/`budget_violation_tol_mwh`
   are dead options. The "hierarchical optimisation" is currently
   *hierarchical simulation with weekly optimisation*. Anyone citing an
   L0-optimised maintenance/energy-budget capability would be wrong.
2. **No inter-week/inter-day storage continuity** (weeks restart at
   `soc_init`; days are cyclic). Correct for daily-cycling BESS; structurally
   wrong for seasonal storage. The restriction bound covers cost, but
   *utilisation statistics* (cycles, CF) inherit the same bias.
3. **Dense carbon uses a fleet-average emission factor**
   (`compute_avg_emission_factor`: unweighted mean over units with positive
   factors, 0.5 tCO₂/MWh fallback) applied to *total* generation — not the
   dispatch-weighted per-unit sum $\sum_g e_g p_{g,t}$, which the UC schedule
   would readily support. On fleets with heterogeneous factors (gas + coal +
   biomass) the primary estimator itself carries a bias the three error bounds
   do **not** cover — the most consequential rigor gap in the carbon layer.
   (The UC's `Carbon` objective *does* use per-unit factors, so the data
   exists.)
4. **The Lipschitz constant is heuristic**: $K_w = 10^{-3}\cdot\max_t|\Delta
   P^{gen}_t|$ with a unit-conversion factor asserted, not derived; and the
   storage bound multiplies by $\max(1, N_{stor})$, double-counting fleet size
   already implicit in total throughput. The bound is best read as an
   order-of-magnitude indicator; the built-in cross-validation percentages are
   the honest check.
5. **Sampling normality**: $z\sqrt{\mathrm{Var}}$ assumes approximately normal
   stratum means; with $n_m$ as small as 1–4 (default 4/stratum) the CI is
   nominal, not exact. Deterministic evenly-spaced sampling
   (`select_sample_hours`, stride-based) also makes the "variance" a model,
   not a sampling-distribution fact.
6. **Capacity-factor denominators**: generator CF uses the last week's end
   step (correct only if weeks tile the year, which they do by construction),
   and renewable CF uses $T_{yr}\Delta t$ — consistent, but PV derating in the
   lifecycle layer changes $\bar P$ mid-study while CF still divides by the
   original rating year by year (each year re-reads current `pmax_mw`,
   so this is consistent per-year; across years the denominators differ — a
   footnote for trend plots).
7. **Grid-exchange closure trade-off** (§4.1): after PF convergence the
   reported exchange is the balance residual, so it absorbs every unmodelled
   mismatch (fallback ESS deviations, non-converged component replays). The
   annual balance closes, but anomalies migrate into the import/export series
   instead of the error series — worth remembering when auditing exchange
   plots.
8. **Storage stats ↔ system mapping**: `compute_storage_stats` positions rows
   by in-service order per group and `lifecycle` matches stats to units *by
   name* (`:536`) — name collisions between AC and DC storage would
   cross-credit cycles. Benign on curated cases, fragile on imported data.
9. **Rich components are outside the annual perimeter.** VPPs, microgrids,
   mobile storage, energy routers, and DR trajectories are neither carried by
   `UCSchedule` nor read by the accounting pass (§4.1), and the lifecycle
   layer hard-resets all opt-in UC blocks to defaults (§5.1). At the annual
   and lifecycle levels these component classes are effectively **modelled in
   the physics (PF), invisible in the economics and statistics**. The full
   stage-by-stage propagation matrix is in
   `time_series_power_flow_models.md` §7.1.

### 7.3 Computational performance — judgement

The architecture reflects a correct reading of where the cost lives: MILP
binaries scale with horizon length (so decompose the horizon), AC-OPF
dominates replay (so make it skippable per fidelity tier), and annual results
are aggregation-dominated (so keep aggregation shared and $O(T)$). The
parallel path's guard matrix (SCIP/native-BC concurrent, HiGHS/Gurobi
serialised, ParityIPM forced) is the difference between "parallel in the
happy path" and "parallel and correct" — and it degrades to a *working*
serial loop, never to a race. Remaining headroom, in order of impact:
(i) per-unit emission-weighted carbon (free — data already in the schedule);
(ii) a real L0 MILP + feedback iteration to unlock seasonal-storage studies;
(iii) SOC chaining across days via a cheap forward pass (turn the restriction
into a rolling horizon at negligible cost); (iv) reuse of the day-MILP basis
across days that share commitment patterns (warm-starting day $d{+}1$ from
day $d$'s solution).

### 7.4 Validation surface

`tests/test_multiscale_comprehensive.cpp` exercises the daily (24-step,
convergence + energy assertions) and annual (365 six-hourly blocks ⇒ 8760 h,
feasibility, ENS = 0, finite balance error) chronology on the same rich
system; the UC feature tests (`tests/test_uc_*.cpp`) pin every opt-in block
the weekly/daily solves rely on; `tests/run_gui_server.cpp` wires
`solve_annual_production_simulation` and `run_lifecycle_simulation` into the
dashboard endpoints, which is where the block summaries, PF snapshots, and
bound cross-validations are consumed.

---

## 8. Summary judgement

The annual layer is a well-engineered decomposition machine: a certified
restriction (cyclic SOC) makes the year embarrassingly parallel, three daily
modes trade fidelity for speed along a clean axis, accounting identities close
by construction, and the lifecycle layer is — unusually for this class of
tool — explicit about its own error budget. Its weak points are concentrated
and fixable: the L0 plan and feedback loop are scaffolding rather than
optimisation, storage continuity beyond 24 h/168 h is structurally absent,
the carbon point estimator uses a fleet-average emission factor that its
otherwise careful bounds do not cover, and the accounting/lifecycle perimeter
stops at the classical component set — VPPs, microgrids, mobile storage,
energy routers, and DR are physically present in the PF but absent from the
annual economics and statistics (§7.2 item 9). Most of these are surfaced by
the implementation — feasibility flags, solver-name provenance, balance-error
series, and per-year bound cross-validation appear exactly where the
approximations bite — but the perimeter gap is silent (their energy is
absorbed into the derived grid-exchange series) and must be kept in mind when
studying systems built around those components.

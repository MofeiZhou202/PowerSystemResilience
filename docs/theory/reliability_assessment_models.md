> Documentation Sync (2026-07-12)
> Scope: reviewed against current repository structure, CMake presets/options, and registered test targets.
> Status: implementation-backed reference.
> Source of truth: when text and implementation diverge, treat src/, include/, tests/, and CMake files as authoritative.

# Reliability Assessment — Mathematical Models & Code Review (Canonical Space)

This document is the engineering/math reference and rigor audit for the hybrid
AC/DC **reliability assessment** module. It is the companion/enhancement to
[`network_reconfiguration_models.md`](network_reconfiguration_models.md): network
reconfiguration is the *restoration kernel* that every contingency-based
reliability method invokes, so the two documents describe one continuum —
*reconfiguration* answers "what is the best topology now?", *reliability* answers
"what is the frequency-and-duration-weighted consequence of every contingency,
restored as well as the topology allows?".

It states the **implemented** math model of each method, classifies each piece
as rigorous or heuristic, and records concrete correctness findings. Historical
code-review roadmaps were removed; runtime validity fields and tests are the
scope authority.

Implementation:
`src/reliability/reliability_assessment.cpp` (resolver, NSQ/SEQ Monte Carlo, F&D,
tail risk, distribution indices, deterministic FMEA),
`src/reliability/failure_mode.cpp` (failure-mode catalog, consequence operator,
failure-mode FMEA), `src/reliability/three_stage_reliability.cpp` (LinDistFlow
restoration MILP), `src/reliability/reliability_data.cpp` (IEEE RTS-24 / template
data). Public contracts: `include/hacdcpf/reliability/reliability_assessment.hpp`,
`include/hacdcpf/reliability/failure_mode.hpp`,
`include/hacdcpf/analysis/three_stage_reliability.hpp`.

Rigor tags used throughout: **✓ rigorous** (textbook-exact for its stated
assumptions), **◐ approximate** (sound but with a documented simplification),
**⚠ questionable** (assumption is arbitrary or applied inconsistently),
**✗ incorrect/misleading** (the math does not compute the quantity it claims).

---

## 0. Executive summary of findings

| # | Area | Verdict | Severity | Where |
|---|------|---------|----------|-------|
| F1 | Two-state unavailability / FOR↔λ↔MTTF conversions | ✓ rigorous | — | `reliability_assessment.cpp:70-217` |
| F2 | F&D COPT probability+frequency recursion | ✓ rigorous (Billinton) | — | `reliability_assessment.cpp:2003-2056` |
| F3 | Sequential MC chronological exponential sampling | ✓ rigorous | — | `reliability_assessment.cpp:1646-1678` |
| F4 | MC convergence CoV (std-error-of-mean) | ✓ rigorous | — | `:1296-1304`, `:1751-1760` |
| F5 | IEEE 1366 SAIFI/SAIDI/CAIDI/ASAI | ✓ rigorous | — | `:2165-2186` |
| F6 | FMEA / 3-stage first-order `Σ λ·τ·shed` | ◐ first-order | low | `:3949-3976`, `three_stage…:1229-1234` |
| **F7** | **Three-stage Stage-3 restores the faulted component for the whole repair window** (empirically confirmed, §6) | **✗ misleading** | **high** | `three_stage…:628`, `:637-641`, `:1212-1234` |
| F8 | NSQ tail-risk built on per-state `shed×8760` samples | ✗ misleading | medium | `:1264-1267`, `:1396-1400` |
| F9 | Hybrid consequence engine now enforces AC-branch DC power flow (Kirchhoff via angles) | ✓ fixed | medium | `:3248-3640` |
| F10 | MC unavailabilities bypass the unified resolver; per-method missing-data defaults diverge | ⚠ inconsistent | medium | `:959-1135`, `:1459-1584` vs `:2413-2428` |
| F11 | FMEA event duration = `τ_sw + full MTTR` (switching time double-counted) | ✓ fixed (τ_rep = MTTR − τ_sw, see §5) | — | `run_distribution_fmea` stage loop |
| F12 | FMEA grid-forming/microgrid support modeled as an unbounded slack (rating ignored) | ⚠ over-optimistic | medium | `:3095-3118`, `:2964-2986` |
| F13 | Single load level in NSQ & F&D (no load-duration curve) | ◐ documented | low | `:1199-1208`, `:2049` |
| F14 | Three-stage reactive demand rebuilt from PF=0.9 (ignores `q_mvar`) | ⚠ discards data | low | `three_stage…:548` |
| F15 | Cyber/comm/control modes now drive EENS (comm-loss→frozen setpoint; data-driven derating) | ✓ fixed | low | `failure_mode.cpp:834-940` |
| F16 | Component-importance = co-occurrence attribution, not a Birnbaum/marginal measure | ◐ heuristic | low | `:1282-1287`, `:1360-1390` |
| F17 | `use_importance_sampling` option unimplemented | ⚠ dead option | low | header `:165-167` |

The headline is **F7**: the most physically detailed method (the three-stage
restoration MILP) systematically *under-counts* the energy not supplied during
the repair window, while the simpler FMEA gets that window right. Sections 5–6
develop this.

---

## 1. Notation

Per component $c$ and failure mode $m$:

- $\lambda_c$ — failure frequency (occ/yr); $r_c$ — mean repair/recovery time (hr).
- $U_c$ — steady-state forced unavailability; $\text{MTTF}_c,\text{MTTR}_c$ (hr).
- $\mu_c = H/r_c$ — repair rate (repairs/yr), with reporting horizon $H$ (8760 hr, or 8736 for the IEEE-RTS week-aligned year).
- Active-on-demand mode: $\nu_d$ demands/yr, $p_d$ failure probability/demand.
- Network consequence of a (failed) state: $S(\cdot)$ = total load shed (MW), $s_i(\cdot)$ = nodal shed at bus $i$.
- System metrics: $\text{EENS}$ (MWh/yr), $\text{EDNS}$ (MW), $\text{LOLE}$ (hr/yr), $\text{LOLF}$ (occ/yr), $\text{PLC}/\text{LOLP}$ (–).

The reliability evaluators **never read the dispatch objective value**; they read
only $S$ and $s_i$. The objective is rigged (Section 7) so the consequence engine
returns the *minimum* feasible shed.

---

## 2. The shared parameter model (resolver)  ✓ F1

`resolve_reliability_params` (`reliability_assessment.cpp:35-217`) maps the
heterogeneous case fields into a canonical $\{\lambda, r, U, \text{MTTF}\}$ tuple.
All conversions are the exact two-state-Markov forms and are **rigorous**:

$$
U=\frac{\lambda}{\lambda+\mu},\quad \mu=\frac{H}{r}
\;\Longleftrightarrow\;
U=\frac{\lambda r}{\lambda r + H}.
$$

From an explicit MTTF (or legacy MTBF with declared convention):
$\lambda=H/\text{MTTF}$, and (with repair $r$) $U=\dfrac{r}{\text{MTTF}+r}$ —
algebraically identical to the line above, confirmed in code by computing
$\lambda$ then overwriting $U$ with the MTTF form (`:121-156`).

From a forced-outage rate $f$ (= $U$) and repair $r$ (`:159-168`):

$$
\text{MTTF}=\frac{r(1-f)}{f},\qquad
\lambda=\frac{f}{(1-f)\,r}\,H .
$$

Active-on-demand equivalent annual frequency (`:97-100`):
$\lambda^{\text{act}} = \nu_d\,p_d$, then folded into the same
frequency–duration form using the recovery time as $r$.

**Provenance honesty (good):** missing/zero/non-finite inputs are treated as
"not provided"; under `StrictCaseDataOnly` nothing is invented, the `data_source`
is tagged `case|template|default|missing`, and the ambiguous MTBF-as-MTTF
assumption emits a warning (`:143-146`). Domain guards reject $f\notin(0,1)$,
$p_d\notin(0,1]$, etc.

> ⚠ **F10 — the resolver is not actually the single source of truth for the
> simulators.** The NSQ and SEQ Monte-Carlo paths compute unavailabilities and
> MTTF/MTTR *inline* (`:959-1135`, `:1459-1584`) and only call the resolver for
> the strict-mode *mask* (`mc_component_has_case_data`) and the data-quality
> report. When case data is present the inline math matches the resolver, but the
> **missing-data fallbacks diverge across methods** for the *same* component:
>
> | undocumented AC branch | effective $\lambda$ used |
> |---|---|
> | NSQ inline default | $U=0.01 \Rightarrow$ implied $\lambda$ ≈ 8.8 occ/yr at $r{=}10$h… i.e. an *unavailability* of 0.01, far above |
> | FMEA resolver default | $\lambda=0.35$, $r=10 \Rightarrow U\approx4\times10^{-4}$ |
> | three-stage default | $\lambda=k\text{DefaultFailureRate}=0.10$ (`three_stage…:43`) |
>
> Three methods, three different implied risks for one undocumented branch. The
> header advertises "one canonical parameter set so that every method consumes
> identical lambda/repair/unavailability" — that contract holds for *present*
> data only. **Fix:** route every per-component $\{U,\text{MTTF},\text{MTTR}\}$ in
> NSQ/SEQ through `resolve_reliability_params` with the same per-kind template
> table FMEA uses.

---

## 3. The consequence-engine fidelity ladder

Every method reduces to "apply failures → minimize load shed on the surviving
network." Three different physics models are used, and **which one runs depends on
the system and the method**, not on a user choice. This is the deepest
cross-cutting issue, so it is stated once here and referenced later.

| Engine | Physics | Used by | Fidelity |
|---|---|---|---|
| **DC-OPF** `solve_dc_opf` | DC power flow (B·θ), thermal limits, load shed; **no V/Q** | NSQ/SEQ/FMEA on **AC-only** systems (`:838`, `evaluate_failed_network_state:3847`) | enforces Kirchhoff voltage law (angles) |
| **Hybrid network LP** `evaluate_hybrid_fmea_network_lp` | AC branches enforce **DC power flow** $P_f=B(\theta_i-\theta_j)$ (angles); zero-impedance edges (switch/breaker/transformer) enforce $\theta_i=\theta_j$; DC side + VSC/DC-DC transfers are transportation | every method on **hybrid AC/DC** systems (`:3248-3640`) | DC-PF on the AC subnetwork; DC/converter transfers transport-bound |
| **LinDistFlow MILP** `solve_stage_milp` | linearised DistFlow: $v_j=v_i-2(rP+xQ)$, V-bounds, thermal, radiality | three-stage AC sub-network only (`three_stage…:367-1137`) | most detailed (has voltage); DC side falls back to connectivity |

> ✓ **F9 (fixed).** The hybrid path (used whenever a DC bus/branch/VSC/DCDC/DC-storage/DC-PV
> exists) now enforces **Kirchhoff's voltage law on the AC subnetwork**: each AC
> branch adds a bus-angle-difference flow definition $P_f=B(\theta_i-\theta_j)$ with
> $B=1/\max(|x|,10^{-4})$, and zero-impedance edges (switch/breaker/transformer)
> collapse to $\theta_i=\theta_j$ (equipotential bus merge, flow free). Radial
> networks reproduce the previous transport answer exactly (unique flow); **meshed
> networks now constrain loop flows**, so the engine no longer routes power along
> paths a real network cannot sustain. Load-shed slack keeps the LP feasible, and
> the `conservative_hybrid_shed` fallback is preserved. The DC subnetwork and the
> VSC / DC-DC transfers remain transportation-bound (voltage-source converters set
> their own terminal, so a transport model of the converter transfer is
> appropriate). Fidelity is now **monotone**: adding a DC bus to an AC case keeps
> DC-PF physics on the AC side rather than relaxing to transportation.

**Connection to reconfiguration.** The repair-stage search in FMEA
(`evaluate_contingency_stage:3724-3814`) and all three stages of the three-stage
MILP are exactly the ONR problem of
[`network_reconfiguration_models.md`](network_reconfiguration_models.md) §1–§6,
specialized to *min-shed* with the faulted element forced open. The three-stage
MILP **is** the LinDistFlow ONR (G2 commodity-flow radiality, G3 power balance,
G4 voltage big-M, G5 thermal, G7 switch budget) solved per restoration stage.
Reliability is therefore "ONR under the contingency set, weighted by
$\lambda$ and stage durations."

---

## 4. Probabilistic adequacy methods

### 4.1 Non-Sequential Monte Carlo  (`run_nonsequential_mc:913-1403`)

**Model.** State sampling: each component independently down with prob $U_c$
(Bernoulli, `sample_state:886-895`). Evaluate the state's shed $S$, accumulate
$\text{EDNS}=\frac1n\sum S$, then

$$
\text{EENS}=\text{EDNS}\cdot H,\qquad
\text{LOLE}=\frac{\#\{\text{loss states}\}}{n}\,H,\qquad
\text{PLC}=\frac{\#\{\text{loss states}\}}{n}.
$$

**Verdict.** The estimator is the standard state-sampling estimator and is
**✓ rigorous** for a *single load level* with independent components. State dedup
(packed-bitset key, `:272-293`), N-0 caching (`:1214`), strict-mode masking
(`:1157-1163`), and base-out-of-service zeroing (`:1149-1152`) are all correct.

**Convergence ✓ F4.** With per-sample variance $\sigma^2$ of $S$,

$$
\text{CoV}=\frac{\sigma}{\bar S\sqrt n}=\frac{\text{SE}(\bar S)}{\bar S},
$$

i.e. the coefficient of variation of the *mean estimator* (`:1298-1304`). Correct
Billinton stopping rule. (Uses the population variance $\frac1n\sum S^2-\bar S^2$
rather than the unbiased $\frac{1}{n-1}$ form — negligible.)

> ◐ **F13.** Load is fixed at the base level $\times$ `load_scale_factor`; there is
> no load-duration curve. NSQ-EENS is therefore "EENS at one load level," not the
> chronological annual integral. This is the conventional adequacy simplification,
> but it should be read as conservative-at-peak / optimistic-at-base depending on
> the scale chosen.

> ✗ **F8 — NSQ tail risk is built on the wrong distribution.** Each sample pushes
> `dns*8760` as one "annual EENS" sample (`:1264-1267`) and VaR/CVaR are taken
> over those (`:1396-1400`). But annual EENS is a *sum over 8760 correlated hours*;
> its distribution is tight (CLT). The per-state $S\times H$ distribution is wildly
> over-dispersed (most states are N-0 → 0; a few severe states → huge), so the
> reported VaR/CVaR describe "one random hour annualized," not the annual risk.
> The percentiles/VaR are essentially the per-state shed distribution scaled by
> $H$. **Fix:** for non-sequential sampling, bootstrap-aggregate states into
> synthetic years before computing tail metrics, or restrict VaR/CVaR to the
> sequential method (where `annual_eens` is a true per-year sum and the metric *is*
> valid).

> ◐ **F16.** "Critical components" rank by `loss_weighted_risk` = the share of
> loss-state shed observed while that component is down (`:1282-1287`). In a
> multi-failure state the *entire* state shed is attributed to *every* down
> component, so contributions sum to more than 100% and this is a co-occurrence
> attribution, not a Birnbaum importance $\partial\,\text{EENS}/\partial U_c$.
> Fine as a ranking; do not read it as marginal risk.

### 4.2 Sequential Monte Carlo  (`run_sequential_mc:1405-1857`)

**Model.** Per component, alternate up/down sojourns drawn by inverse-transform
sampling of the exponential (`:1646-1678`):

$$
T^{\uparrow}=-\text{MTTF}\,\ln u,\qquad T^{\downarrow}=-\text{MTTR}\,\ln u,\quad u\sim\mathcal U(0,1).
$$

Hourly states feed the consequence engine with the chronological load scale
$\text{(profile)}\times\text{(stress)}$ (`hourly_load_scale:558-567`). Per-year
$\text{EENS}=\sum_h s_h$, $\text{LOLE}=\#\{\text{loss hours}\}$, and **LOLF** =
count of $0\to1$ transitions in the hourly loss flag (`count_loss_events:898-905`).

**Verdict ✓ F3.** This is the rigorous reference method: it captures chronology,
the load-duration curve, *and* loss frequency. Its `annual_eens`/`annual_lole`
are genuine per-year samples, so its tail-risk metrics (F8) **are** valid.

> ◐ Minor: durations are rounded to integer hours with `max(1,·)` (up-time
> `round`, down-time `ceil`, `:1661`/`:1666`). Every event lasts ≥ 1 hr, which
> slightly inflates very short (e.g. cyber-recovery) outages and discretizes
> sub-hour MTTRs. Asymmetric round/ceil adds a tiny duration bias. Negligible at
> annual scale.

### 4.3 Frequency & Duration / COPT  (`run_frequency_duration_analysis:1924-2059`)

**Model.** Recursive cumulative capacity-outage probability table over generators
only. For each unit (capacity $C$, availability $p$, unavailability $q$, failure
rate $\lambda$, in MW / per-hour) the cumulative-state recursion is

$$
P_{\text{new}}(X)=p\,P(X)+q\,P(X-C),
$$
$$
F_{\text{new}}(X)=p\,F(X)+q\,F(X-C)+\lambda\,p\,\big[P(X-C)-P(X)\big].
$$

**Verdict ✓ F2.** Both lines are the exact Billinton–Allan recursive COPT build:
the frequency increment $\lambda p[P(X{-}C)-P(X)]$ is precisely the
boundary-crossing rate contributed by the new unit failing out of its up-state
across the level $X$ (`:2017-2025`). Indices:

$$
\text{LOLP}=P(\text{outage}>\text{reserve}),\quad
\text{LOLF}=F(\cdot)\,H,\quad
\text{LOLE}=\text{LOLP}\,H,\quad
\text{LOLD}=\text{LOLE}/\text{LOLF}.
$$

**Scope.** Generation adequacy only (HL-I): no network, transmission, or load
model; single load level; 10 MW discretization with integer capacity rounding
(`:2004`); the reserve→cumulative index has a one-step ($\le$10 MW) coarseness
(`:2043`). All documented; treat as a fast screening bound, not a network EENS.

---

## 5. Deterministic FMEA  (`run_distribution_fmea:3858-4028`)

**Model.** Enumerate every in-service component as an N-1 contingency $k$ with
frequency $\lambda_k$. Two stages are evaluated by the consequence engine:

- **Switching stage** (duration $\tau^{sw}_k$ = `switching_time_hr`, default 0.5 hr):
  fault isolated, emergency sources and islanding applied, **no** tie
  reconfiguration. Shed $S^{sw}_k$.
- **Repair stage** (duration $\tau^{rep}_k=\text{MTTR}_k-\tau^{sw}_k$, F11 fix —
  the event lasts MTTR in total and the two stages are disjoint windows): fault
  still out, plus a greedy ≤2-action switch/branch reconfiguration search.
  Shed $S^{rep}_k$. The physical repair horizon (full MTTR) is still used for
  storage-energy feasibility; only the frequency weighting uses the stage
  duration.

Aggregation (`:3949-3976`):

$$
\text{EENS}=\sum_k \lambda_k\big(S^{sw}_k\tau^{sw}_k+S^{rep}_k\tau^{rep}_k\big),\qquad
\text{LOLE}=\sum_k\lambda_k\big(\tau^{sw}_k\mathbb 1_{S^{sw}_k>0}+\tau^{rep}_k\mathbb 1_{S^{rep}_k>0}\big),
$$
$$
\text{LOLF}=\sum_k\lambda_k\,\mathbb 1_{S^{sw}_k>0\,\lor\,S^{rep}_k>0}.
$$

**Verdict ◐ F6.** This is the standard first-order analytical FMEA expectation:
valid when contingencies are rare and non-overlapping (no N-2), each occurring
$\lambda_k$ times/yr with the staged shed. The two-stage decomposition correctly
keeps the faulted component **out during the entire repair window** — so for load
that switching cannot restore, $S^{rep}_k\cdot\text{MTTR}_k$ is charged, which is
physically right (contrast F7). Sorting, nodal accumulation, and CIF/CID→SAIFI
are consistent.

> ✓ **F11 (fixed).** The event now lasts MTTR in total: switching shed is
> charged for $\tau^{sw}$ and reconfigured shed for
> $\tau^{rep}=\max(0,\text{MTTR}-\tau^{sw})$ (with $\tau^{sw}$ clamped to
> $\le$ MTTR), matching the textbook and three-stage conventions. The physical
> repair horizon (full MTTR) is retained for storage-energy limits. Note this
> slightly LOWERED historical FMEA EENS values (≈5% at MTTR=10 h) relative to
> the old $\tau^{sw}+\text{MTTR}$ accounting.

> **Level-1 cyber-physical conditioning (optional).** When
> `FMEAOptions::cyber_physical.enabled` is set, every stage above is evaluated
> per cyber class (automation available / unavailable) and mixed with the
> scalar automation availability $A_k$; the class switching times replace
> `switching_time_hr`, and the automation-unavailable class keeps crew-based
> switch reconfiguration in the repair stage but freezes DER/storage/
> grid-forming/islanding dispatch. Model, decomposition, and metric definitions:
> archived [cyber-physical fidelity ladder](../archive/theory/cyber_physical_reliability_extension.md)
> §4.1/§6 (implemented at Level 1 for this method only).

> ⚠ **F12.** Grid-forming support is modeled by promoting the device's bus to a
> **SLACK external grid** (`mark_island_anchor:2931-2947`) which
> `add_external_grid_dispatch_sources` then backs with
> $p_{\max}=\max(1000,\,2\times\text{demand})$ MW (`:2964-2986`) — **the converter's
> own MVA rating is never imposed**. A single small grid-forming VSC therefore
> fully energizes its island regardless of its rating, so post-fault support is
> over-credited. The header already warns this "can overestimate available
> support" (`:585-587`); the fix is to cap the anchor injection at the device
> rating (and add an islanded power balance), not the heuristic 2×demand.

> ◐ The repair reconfiguration is a **greedy, OPF-budget-capped** enumeration
> (`max_repair_opf_calls=200`, `:3756-3812`), not a proven optimum, and is flagged
> via `repair_search_truncated`. It enumerates AC switches/branches only (no DC
> breakers/branches/DCDC/VSC topology). So FMEA restoration is a heuristic upper
> bound on served load, whereas the three-stage method solves a MILP — another
> cross-method inconsistency.

---

## 6. Three-stage restoration MILP  (`three_stage_reliability.cpp`)

**Model.** Per branch fault $k$, three LinDistFlow MILPs minimize $\sum_i p^{sh}_i$
over the intervals $[0,\tau_{SW}]$, $[\tau_{SW},\tau_{TP}]$, $[\tau_{TP},\tau_{RP}]$.
The MILP (documented at `three_stage…:282-332`) is exactly the ONR LinDistFlow of
the reconfiguration doc: active/reactive balance, voltage drop
$v_j=v_i-2(rP+xQ)$ with big-M decoupling on open branches, thermal big-M, a
single-commodity radial-forest, switch-count budget $K^{sw}$, and continuous shed
$p^{sh}_i\in[0,p_{d,i}]$. As a **formulation** this is the most rigorous engine in
the module (✓ structure). The stage-conditioned topology is:

| Stage | interval / duration | faulted element | tie switches |
|---|---|---|---|
| 1 (isolation) | $[0,\tau_{SW}]$, $\tau^{iso}$ | **open** (`z_k=0`) | open |
| 2 (reconfig)  | $[\tau_{SW},\tau_{TP}]$, $\tau^{sw}$ | **open** (`z_k=0`) | **closeable**, $\le K^{sw}$ |
| 3 (post-repair) | $[\tau_{TP},\tau_{RP}]$, $\tau^{rep}\approx\text{MTTR}$ | **CLOSED — restored** (`:628`) | forced open → nominal (`:637-641`) |

EENS per load (`:1229-1234`):
$\text{ENS}_i=\sum_k\lambda_k\big(s^{1}_i\tau^{iso}_k+s^{2}_i\tau^{sw}_k+s^{3}_i\tau^{rep}_k\big)$.

> ✗ **F7 — the repair-window duration multiplies the wrong (already-restored)
> topology.** The interval $[\tau_{TP},\tau_{RP}]$ is the *repair window*: the
> faulted component is being repaired and is therefore **still out** until the
> instant $\tau_{RP}$. Yet Stage 3 sets the faulted branch back **in service**
> (`if (br.failed && stage < 3)` is false at stage 3, `:628`; mirrored in the DC
> fallback `:1057-1058`,`:1064-1065`) and reopens the ties to nominal. So Stage 3
> evaluates the **fully repaired** network — which yields $s^{3}_i\approx 0$ — and
> then multiplies that ≈0 shed by the **long** repair duration $\tau^{rep}\approx
> \text{MTTR}$.
>
> Consequence: load that Stage-2 switching **cannot** restore (radial feeder, no
> back-feed path, insufficient tie capacity) is charged only the Stage-2 duration
> $\tau^{sw}=\tau_{TP}-\tau_{SW}\approx\tfrac{1}{60}$ hr (≈ 1 min), instead of the
> repair duration MTTR. For a radial load with no tie, the correct
> $\text{ENS}=\lambda\,\text{Load}\cdot\text{MTTR}$; the code returns
> $\lambda\,\text{Load}\cdot(\tau^{iso}+\tau^{sw})\approx\lambda\,\text{Load}\cdot\tfrac1{30}$ hr
> — an **under-estimate of ~30–150×** for unrestorable load.
>
> Root cause: Stage 3 is labeled "post-repair" but is assigned the repair-window
> *duration*. The window $[\tau_{TP},\tau_{RP}]$ is the time *during which* the
> component is out being repaired, so Stage 3 should keep `z_k=0` (fault out) with
> the Stage-2 reconfiguration **held**, and only return to nominal at the instant
> repair completes (a zero-duration boundary). **Fix:** in Stage 3 force the
> faulted element open (same as Stage 2) and keep the reconfigured ties closed;
> $s^{3}_i$ then equals the genuinely unrestorable shed and the
> $s^{3}_i\cdot\text{MTTR}$ term becomes correct. Note FMEA (§5) already does this
> correctly — the simpler method is the more accurate one here.
>
> **Empirical confirmation.** A radial 1-branch feeder (10 MW source → 1 MW load,
> no tie, $\lambda=1$/yr) where the single branch fault *topologically isolates*
> the load was run through `run_three_stage_reliability_from_string` for
> MTTR ∈ {1, 10, 100, 1000} h (test `[reliability][three_stage][f7]` in
> `tests/test_three_stage_reliability.cpp`):
>
> | MTTR (h) | shed₁ (kW) | shed₂ (kW) | shed₃ (kW) | **EENS (kWh/yr)** | textbook $\lambda L\,\text{MTTR}$ |
> |---:|---:|---:|---:|---:|---:|
> | 1 | 1000 | 1000 | **0** | **33.333** | 1 000 |
> | 10 | 1000 | 1000 | **0** | **33.333** | 10 000 |
> | 100 | 1000 | 1000 | **0** | **33.333** | 100 000 |
> | 1000 | 1000 | 1000 | **0** | **33.333** | 1 000 000 |
>
> EENS is flat at $33.333=\lambda\,L\,(\tau^{iso}+\tau^{sw})=1000\cdot\tfrac{2}{60}$
> kWh/yr across a 1000× MTTR sweep, and $s^{3}\equiv0$. The repair window
> contributes nothing; the under-estimate is ≈300× at MTTR=10 h and ≈30 000× at
> MTTR=1000 h. (The pre-existing "longer MTTR raises EENS" test passes only because
> it uses a *capacity* shortfall, $s^{3}\ne0$, which keeps a residual repair-window
> term — that test cannot detect this topology-isolation case.)

> ⚠ **F14.** Reactive demand is **rebuilt** as $q_{d,i}=p_{d,i}\tan(\arccos 0.9)\approx
> 0.4843\,p_{d,i}$ (`:548`), discarding each load's actual `q_mvar`. The 0.9-PF
> assumption is applied uniformly even when measured Q is available.

> ◐ Hybrid scope: DC buses/branches/VSC are **not** in the AC MILP; VSC/SOP
> setpoints are fixed at 0 (`:1216-1221`), and `r.ok` is forced false for any
> DC/VSC case (`:1308`). ✓ **DC power flow (default on):** the DC subnetwork is a
> **DC LinDistFlow LP** (per-bus voltage bounds $v\in[v_{\min}^2,v_{\max}^2]$,
> resistive drop $v_j=v_i-2rP$, per-branch thermal limits, DC sources, and VSC
> transfers budgeted by the AC component surplus), so DC line congestion and
> voltage violations shed load the old aggregate check missed. Set
> `include_dc_power_flow=false` to force the legacy capacity fallback (also used
> automatically if the DC LP fails to solve). `dc_power_flow_enforced` and
> `model_scope` (`…+dc-lindistflow` vs `…+dc-connectivity-fallback`) report which
> is active. Honestly flagged in `validity`/`model_scope` (`:1280-1289`).
> Solver hardening (bound/integrality/residual post-checks with conservative
> full-shed fallback, `:957-1026`) is solid.

> ◐ Stage boundary times $\tau_{SW}=1$ min, $\tau_{TP}=2$ min are **global
> constants** (`:39-41`), not per-device, and the default repair when MTTR is
> absent is 1 hr — short for distribution assets.

> ✓ **Fault set (extended).** N-1 enumeration covers **ACBranch** and **DCBranch**
> outages by default; **generator, transformer, VSC/DC-DC converter, AC switch,
> and AC/DC circuit-breaker** outages are enumerated when their opt-in flags
> (`include_generator_faults`, `include_transformer_faults`,
> `include_converter_faults`, `include_switch_faults`) are set. Converter and
> DC-breaker faults act through the DC connectivity fallback (the faulted coupling
> and its transfer capacity are dropped); AC switch/breaker faults become
> forced-open AC restoration edges. Default off preserves the historical
> branch-only enumeration, so existing SAIFI/EENS are unchanged unless enabled.

---

## 7. The min-shed objective rig  ◐

`reliability_shedding_voll` (`:252-270`) sets the value of lost load to
$1000\times$ the worst source marginal cost, clamped to $[10^5,10^7]$ \$/MWh, so
the LP/OPF is a lexicographic *minimum-load-shedding* problem (shed only when
physically forced; source cost is a tie-breaker). Because the evaluators read only
$S,s_i$ and never the objective value, inflating VOLL changes *which* min-shed
dispatch is chosen but not the reported shed — **correct and well-reasoned**. Edge
case: if a source marginal exceeds $10^4$ \$/MWh, the $10^7$ cap can erode the
intended $1000\times$ dominance margin; in practice generator marginals are far
below this.

---

## 8. Failure-mode model (`failure_mode.cpp`)  ◐ F15

A rich component expands into multiple **failure modes** (passive/active ×
physical/cyber/protection/comm/measurement), each resolved through §2 and mapped
by a **consequence operator** $\Phi_m$ to network mutations
(`build_consequence_patch:764-928`). `run_failure_mode_fmea:1167-1307` then sums
the same first-order $f_m\cdot\text{dur}_m\cdot S_m$ with
$\text{dur}_m=\tau^{iso}+\tau^{sw}+r_m$ and $f_m=\lambda$ (passive) or $\nu_d p_d$
(active).

**Verdict.** The taxonomy, provenance, and "honest support gate" (modes the
steady-state engine cannot represent are reported `unsupported`, not silently
applied — `:829-851`) are a genuine strength. Scope after the F15 wiring:

- `ForcedOutage`, `Derating`, load-point shed, aggregated-source outage,
  breaker `FailToTrip→ProtectionZoneExpansion`, **control-unavailable /
  setpoint-frozen (pinned setpoint), and communication-loss on a dispatchable
  converter/DER (frozen dispatch → possible shed)** now drive EENS. A
  communication-loss on a *non-dispatchable* target (switch/breaker), pure
  `MeasurementBias`, and `FailToClose` still resolve to zero steady-state shed —
  correctly, since those are restoration-path / state-estimation effects with no
  first-order shed signature (they remain honestly `unsupported`).
- ✓ **F15a.** Communication loss on a controllable converter now maps to
  loss-of-dispatch (`RemoveControllability`): the device holds its last setpoint
  and cannot re-dispatch, so an island that relied on its flexible infeed sheds
  (`build_consequence_patch` `CommunicationLoss` case).
- ✓ **F15b.** `Derating` severity is **data-driven**: each derating mode carries a
  `residual_capacity_factor` (surviving fraction, e.g. thermal 0.75, cooling 0.70,
  converter power-stage 0.70) instead of a uniform 0.5, and the consequence mapper
  reads it. `derate()` now scales **both** transfer directions of a bidirectional
  converter (`pmax` and `pmin`), so a converter feeding a DC island is actually
  capacity-limited (previously derating only touched `pmax`, a no-op for AC→DC).
- `ProtectionZoneExpansion` opens **every** edge incident to `bus_from`
  (`expand_protection_zone:1014-1049`) — a conservative radial approximation that
  is exact only for radial feeders; meshed buses are mis-handled either way.
- \u2713 **Multi-mode (N-2) co-failures** are now enumerated on demand
  (`FailureModeFMEAOptions::max_order >= 2`): pairs of supported modes on
  **distinct** components are composed via `compose_consequence_patches` (correct
  outage-dominates-derating precedence, `:1086-1137`; hard forced-open/closed
  conflicts skipped), evaluated jointly, and weighted by the independent
  second-order overlap $U_i U_j$ (EENS $+= U_i U_j\cdot 8760\cdot S_{ij}$). A
  `min_pair_unavailability` floor and `max_pairs_evaluated` cap keep the
  $O(M^2)$ enumeration tractable; `co_contingencies[]` reports the ranked joint
  states. Default `max_order = 1` (single-mode) is unchanged.
- Independence across the *several modes of one component* is assumed (their EENS
  contributions add), which double-counts to second order — fine when modes are
  rare.

---

## 9. Metric definitions (as implemented)

$$
\text{EDNS}=\tfrac1n\textstyle\sum S\ \text{(MW)},\quad
\text{EENS}=\text{EDNS}\cdot H,\quad
\text{LOLE}=\text{PLC}\cdot H,\quad
\text{PLC}=\Pr[S>\varepsilon],
$$
with curtailment threshold $\varepsilon=$ `curtail_threshold_mw` (0.01 MW).
SEQ adds $\text{LOLF}=\mathbb E[\,$0→1 loss transitions/yr$\,]$.

**IEEE Std 1366 ✓ F5** (`compute_distribution_indices:2065-2191`):

$$
\text{SAIFI}=\frac{\sum_i \text{CIF}_i N_i}{\sum_i N_i},\quad
\text{SAIDI}=\frac{\sum_i \text{CID}_i N_i}{\sum_i N_i},\quad
\text{CAIDI}=\frac{\text{SAIDI}}{\text{SAIFI}},\quad
\text{ASAI}=1-\frac{\text{SAIDI}}{H},\quad \text{ASUI}=1-\text{ASAI}.
$$

Hybrid `[AC | DC]` nodal layout is handled; DC customers are appended. ⚠ When
`n_customers` is absent, customers default to $10\times$MW (`:2106`,`:2114`,`:2149`),
turning SAIFI/SAIDI into load-weighted (not customer-weighted) indices — a
documented proxy, but the 10/MW constant is arbitrary and should be a parameter.

**Tail risk** (`compute_tail_risk:1863-1918`): VaR = ascending
$\lceil c\,n\rceil$-quantile; CVaR = mean of the upper tail from VaR onward
(expected shortfall). The estimator is correct; its **validity depends entirely
on the input** — valid for SEQ `annual_eens`, invalid for NSQ (F8). The CVaR
includes the VaR index itself (slight optimistic-tail bias) and percentiles use
floor indices without interpolation (minor).

---

## 10. What is rigorous vs. what to fix

**Rigorous and trustworthy (keep):** the two-state parameter conversions (F1); the
Billinton COPT probability+frequency recursion (F2); sequential MC chronology and
its LOLF (F3); the MC CoV stopping rule (F4); IEEE-1366 indices (F5); the
LinDistFlow MILP *formulation* and its solver post-checks; the min-shed objective
rig (§7); the consequence-patch provenance/support-gate discipline.

**Priority fixes (math correctness):**

1. **F7 — three-stage repair window.** Hold the faulted element open and the
   reconfiguration closed through Stage 3 so the MTTR duration multiplies the
   genuinely-unrestorable shed. *This is the single largest accuracy error in the
   module and it makes the most-detailed method the least accurate for
   non-restorable load.*
2. **F8 — NSQ tail risk.** Stop feeding `shed×8760` as annual samples; restrict
   VaR/CVaR to the sequential method or aggregate states into synthetic years.
3. **F10 — parameter unification.** Make NSQ/SEQ consume
   `resolve_reliability_params` (values, not just the strict mask) so missing-data
   defaults match FMEA/three-stage.

**Secondary (fidelity/consistency):**

4. **F9** ✓ *done* — the hybrid consequence engine now enforces DC-power-flow
   physics on the AC subnetwork (branch angle-difference flow definitions;
   zero-impedance edges equipotential), so fidelity is monotone with system
   richness. DC/converter transfers remain (appropriately) transportation-bound.
5. **F12** — cap FMEA grid-forming/microgrid support at the device rating with an
   islanded power balance, instead of a 2×demand slack.
6. **F11/§6** ✓ *done* — FMEA now uses the three-stage convention
   ($\tau^{rep}=\text{MTTR}-\tau^{sw}$); one duration convention across methods.
7. **F14** — use measured `q_mvar` in the three-stage MILP when available.
8. **F15** ✓ *done* — control-unavailable / setpoint-frozen and communication-loss
   on a dispatchable converter/DER now wire into the shed engine (frozen dispatch
   → possible shed), and derating severity is data-driven (`residual_capacity_factor`,
   with bidirectional converter derating fixed). Measurement-bias, comm-loss on
   non-dispatchable devices, and fail-to-close remain honestly `unsupported`
   (restoration / state-estimation effects, no first-order shed).
9. **F13/F16/F17** — expose the load-level/LDC assumption; relabel
   "critical components" as co-occurrence share (or implement a Birnbaum measure);
   either implement or remove the importance-sampling option.

**One-line consistency note for the GUI/API:** the same hybrid case can produce
materially different EENS under NSQ, FMEA, failure-mode FMEA, and three-stage —
not from sampling noise but because each invokes a *different consequence engine,
parameter-default set, and restoration optimality*. Surface the active
`model_scope`/`validity` flags (already populated) next to every reported index so
the numbers are never compared across methods as if commensurable.

---

## 11. Cross-reference

- Reconfiguration / restoration kernel and the canonical LinDistFlow ONR:
  [`network_reconfiguration_models.md`](network_reconfiguration_models.md).
- Failure-mode implementation: `src/reliability/failure_mode.cpp` and the
  registered reliability tests.
- Per-method capability declarations are emitted at runtime in
  `ReliabilityResult::validity` / `FMEAResult::validity` /
  `ThreeStageReliabilityResult::validity` — treat them as the authoritative scope
  statement for any exported number.

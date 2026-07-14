> Document status (2026-07-14)
> Scope: theoretical design & decision document. **Level 1 (§4.1, §6) is now
> implemented for the deterministic FMEA method** — `CyberPhysicalFMEAOptions`
> in `include/hacdcpf/reliability/reliability_assessment.hpp`, evaluated in
> `run_distribution_fmea`, exposed via the GUI server (`cyber_physical` request
> block) and the reliability panel. Levels 2–4 remain design-only. Companion to
> [`reliability_assessment_models.md`](reliability_assessment_models.md)
> (the implemented physical baseline, whose §5 documents the shipped Level-1
> conditioning). Source of truth: src/, include/, tests/.
>
> Implementation notes where the shipped Level 1 refines this text:
> the automation-unavailable class **credits crew-based switch reconfiguration
> during the repair stage** (manual restoration, per §4.1's c₂ table) while
> freezing DER/storage/grid-forming/islanding dispatch;
> `manual_switching_time_hr` is clamped to ≥ the automatic time;
> `delta_protection_misoperation_mwh_yr` is an API placeholder (always 0 at
> Level 1 — protection misoperation belongs to the failure-mode FMEA, §4.2 C3).

# Cyber-Physical Reliability Assessment — Theoretical Extension & Decision Guide

This document answers one question: **what does it take — in theory, data, and
code — to upgrade the current *physical* reliability assessment of the hybrid
AC/DC distribution system to a *cyber-physical* assessment, and which of the
possible depths is worth building?**

It is organized as a fidelity ladder (Levels 0–4). Each level states its
mathematical model, the new data it demands, the code touch points, and the
engineering questions it can answer that the previous level cannot. Section 9
condenses this into a decision matrix with a recommendation.

Modeling-choice tags reuse the house convention:
**✓ rigorous** (exact for its stated assumptions), **◐ approximate**
(documented simplification), **⚠ questionable** (assumption needs care),
**✗ out of scope for steady-state adequacy** (would be dishonest to fold into
EENS).

---

## 0. Executive summary

1. **The consequence of most cyber failures is not load shed — it is *time*.**
   A dead feeder terminal unit does not interrupt a single customer; it turns a
   2-minute automatic FLISR restoration into a 60-minute manual one, and it can
   silence the DER support the restoration MILP assumes. The correct theoretical
   entry point is therefore the **stage durations and stage consequence patches**
   of the existing FMEA / three-stage machinery — not a new index and not a new
   solver.

2. **The codebase is already half-prepared.** The failure-mode taxonomy
   (`failure_mode.hpp:90-116`) carries cyber causes (`CyberControl`,
   `Communication`, `Measurement`, `ProtectionLogic`), cyber consequence kinds
   (`ControlUnavailable`, `SetpointFrozen`, `CommunicationLoss`, `FailToTrip`,
   …), a dedicated `cyber_recovery_hr`, and active-on-demand parameters
   $(\nu_d, p_d)$. What is missing is (a) a **cyber layer topology** (there is
   none — confirmed absent from `include/hacdcpf/model/`), (b) the **coupling
   of cyber state into restoration timing and success**, and (c) the
   **physical→cyber back-coupling** (comm nodes are powered by the grid they
   monitor).

3. **Recommended target: Level 1 now, Level 2 as the architecture, Level 3
   through the existing sequential MC only.** Level 4 (adversarial attacks)
   should be kept **out of the reliability indices** and treated with the
   resilience machinery (`src/scenario_generation/typhoon_resilience.cpp`
   precedent), because attacks are not Poisson and folding them into EENS
   destroys the meaning of both.

4. **Prerequisite fixes.** The cyber extension acts almost entirely through
   restoration-window durations and per-method parameter consistency. Two open
   baseline findings corrupt exactly those channels and must land first:
   - **F7** (three-stage Stage 3 evaluates the already-repaired network for the
     repair window) — with F7 open, lengthening the switching stage by a cyber
     failure is *invisible* for unrestorable load, i.e. the cyber delta is
     computed against a broken baseline.
   - **F10** (NSQ/SEQ bypass the unified parameter resolver) — cyber components
     will enter through `resolve_reliability_params`; if the MC paths keep
     inventing their own defaults, physical and cyber parameters will be
     inconsistent across methods from day one.

---

## 1. What "cyber-physical" adds: the interdependency taxonomy

Let $G_p$ be the physical network (the existing `HybridPowerSystem`) and $G_c$
a cyber layer. Four coupling channels exist; a useful extension must decide,
channel by channel, at which level it is represented.

| # | Channel | Direction | Physical meaning | Steady-state representable? |
|---|---------|-----------|------------------|------------------------------|
| C1 | **Monitoring / observability** | cyber → physical | fault indicators, FTU/DTU measurements determine *fault-location time* (isolation stage duration) | ✓ as a stage-duration modifier |
| C2 | **Control / dispatchability** | cyber → physical | remote switching, DER/VSC/ESS setpoint updates, islanding commands; loss ⇒ frozen setpoints, manual switching | ✓ as consequence patches (partially shipped, F15) + duration modifiers |
| C3 | **Protection logic** | cyber → physical | IED/breaker fail-to-trip (zone expansion), nuisance trip (new initiating events), GOOSE/comm-assisted protection schemes | ✓ as active-on-demand modes (taxonomy shipped) |
| C4 | **Power supply of cyber nodes** | physical → cyber | RTUs, switches, base stations are fed from the distribution grid, with battery ride-through $T_b$ | ◐ second order for adequacy; first order only in sequential MC / storms |

Two classical framings map onto this table and are worth naming because the
literature uses them:

- **Direct interdependency** (element-wise): a cyber element's failure directly
  changes the state or controllability of an identified physical element
  (Falahati & Fu, *IEEE Trans. Smart Grid*, 2012). This is C2/C3 and is exactly
  the existing `ConsequencePatch` mechanism.
- **Indirect interdependency** (functional): cyber failures degrade a *function*
  (state estimation, FLISR, protection coordination) whose loss changes system
  behaviour only *conditionally on another event happening* (Falahati & Fu,
  2014). This is C1 and the duration/success channel — the part the codebase
  does not yet have, and the theoretical core of this document (§4).
- The **cyber-physical interface matrix** (Lei & Singh) is the compact bookkeeping
  device for both: a matrix $\Pi \in [0,1]^{|F|\times|K|}$ giving, for each
  cyber-layer state class, the probability that physical consequence class $k$
  results. §4.3 adopts it as the boundary between the (new) cyber evaluation
  and the (existing) physical consequence engine — it is what keeps the two
  solvers decoupled.

**Design principle (inherited from the baseline):** every channel a chosen
engine cannot represent must be reported `unsupported`, never silently zeroed
— the existing honest-support-gate discipline (`failure_mode.cpp`,
`supported_by_selected_consequence_model`) extends unchanged to cyber modes.

---

## 2. Baseline: what the module already does, precisely

From [`reliability_assessment_models.md`](reliability_assessment_models.md):

- **Methods:** non-sequential MC (state sampling), sequential MC (chronological,
  exponential sojourns, valid tail risk), analytical F&D/COPT, deterministic
  two-stage FMEA, three-stage LinDistFlow restoration MILP, failure-mode FMEA
  (N-1 and opt-in N-2 composition).
- **Parameter model:** two-state Markov per failure mode, unified resolver
  $\{\lambda, r, U, \text{MTTF}\}$ with provenance (`case|template|default|missing`),
  active-on-demand $\lambda^{act} = \nu_d\, p_d$, and a `cyber_recovery_hr` that
  substitutes for physical repair on cyber modes
  (`reliability_assessment.hpp:59-98`).
- **Cyber today (Level 0):** cyber failure modes exist *on the physical
  device's own catalog entry* — e.g. "VSC1 communication loss" — and since F15
  they map to real consequence patches: `CommunicationLoss` on a dispatchable
  converter ⇒ `RemoveControllability` (frozen setpoint, possible shed);
  `Derating` with data-driven `residual_capacity_factor`; breaker
  `FailToTrip` ⇒ `ProtectionZoneExpansion`. Modes with no first-order shed
  signature (`MeasurementBias`, `FailToClose`, comm loss on a non-dispatchable
  switch) are honestly `unsupported`.

**The three structural gaps** that make Level 0 "physical with cyber-flavored
modes" rather than cyber-physical:

1. **No cyber topology.** Each cyber mode is an independent coin flip attached
   to one device. In reality one fiber ring, one substation gateway, or the
   DMS front-end is a **common-cause element** for dozens of devices: its
   failure disables FLISR on a whole feeder at once. Independent per-device
   coins strictly *underestimate* the variance and the tail (many small
   independent losses instead of one large correlated loss), and misprice
   redundancy investments (a second gateway looks worthless).
2. **No time channel.** `switching_time_hr` (0.5 h default), $\tau^{iso}$,
   $\tau^{sw}$ are constants (`three_stage…:39-41`,
   `FailureModeCatalogOptions::default_*`). Automation state does not touch
   them, so the primary economic value of distribution automation — SAIDI/EENS
   reduction via faster FLISR — is invisible to the tool, and so is its loss.
3. **No physical→cyber coupling.** Cyber nodes never lose power (C4).

---

## 3. The cyber layer model

### 3.1 Topology ✓

Model the cyber layer as an undirected graph (directed only if asymmetric
channels matter, which for availability they usually do not):

$$
G_c = (N_c, E_c),\qquad
N_c = N_{\text{term}} \cup N_{\text{net}} \cup N_{\text{ctl}},
$$

- $N_{\text{term}}$ — terminal devices bound 1:1 to physical components:
  FTU/DTU on switches and breakers, RTU/gateway per secondary substation,
  DER/VSC/ESS local controllers, smart meters (optional).
- $N_{\text{net}}$ — transport: fiber segments/rings, Ethernet switches,
  routers, wireless base stations, PLC couplers.
- $N_{\text{ctl}}$ — head end: front-end processors, SCADA/DMS/ADMS servers
  (possibly redundant pairs), substation controllers for *distributed*
  automation schemes.

Each cyber component is a **two-state Markov element** with the same canonical
tuple $\{\lambda_c, r_c, U_c\}$ as physical components, resolved by the *same*
`resolve_reliability_params` (reuse, not re-implementation; `r` is
`cyber_recovery_hr` semantics — reboot/repair/failover time). Software common
modes (server OS, DMS application) enter as additional series elements with
their own tuples, ◐ acknowledging that software failures are only
approximately Poisson.

Two couplings annotate the graph:

- **Service binding** $\beta: D \to 2^{N_c}$ — for each physical device $d$
  (switch, breaker, converter, DER), the set of cyber nodes/edges whose joint
  function $d$'s remote operation requires (its terminal + transport path(s) +
  the controlling master). With redundant paths this is not a set but a
  **structure function** (§3.2).
- **Power binding** $\pi: N_c \to B \cup \{\varnothing\}$ — the physical bus
  feeding each cyber node, plus battery ride-through $T_b(n)$ hours (C4).

### 3.2 Function availability as a structure function ✓

For a control action on device $d$ (e.g. "remotely open switch $d$"), define
the Boolean structure function over cyber component states
$x \in \{0,1\}^{|N_c \cup E_c|}$:

$$
\phi_d(x) = x_{\text{term}(d)} \;\wedge\; x_{\text{ctl}} \;\wedge\;
\Big(\bigvee_{p \,\in\, \mathcal P(d)} \bigwedge_{e \in p} x_e\Big),
$$

where $\mathcal P(d)$ is the set of minimal transport paths from $d$'s terminal
to the controlling master. The **on-demand availability** of the function is

$$
A_d = \Pr[\phi_d(X)=1], \qquad X_i \sim \text{Bernoulli}(1-U_i)\ \text{indep.}
$$

Computation options, in increasing generality:

| Method | Exact? | Cost | When |
|---|---|---|---|
| series-parallel reduction | ✓ | trivial | radial comm chains, simple rings |
| minimal cut sets + inclusion–exclusion (or the standard ≤2nd-order bound) | ✓/◐ | $O(\text{cuts})$ | meshed comm, few redundancies — **recommended default** |
| BDD | ✓ | moderate | many shared elements |
| graph-state Monte Carlo | ◐ (sampling) | cheap per state | already free inside NSQ/SEQ MC (§5.2–5.3) |

Because $U_i$ for comm gear is small ($10^{-4}$–$10^{-3}$), the first-order cut
bound $A_d \approx 1 - \sum_{C \in \text{min cuts}} \prod_{i\in C} U_i$ is
accurate and gives per-cut attribution for free (which cut dominates ⇒ where
redundancy pays). ✓ rigorous under independence; correlated failures are
handled where they belong, §5.3/§7.

For a *composite* function like FLISR on contingency $k$, the required set is
the conjunction over the devices the restoration plan actually uses:
fault indicators upstream of $k$ (C1), the isolating switches, the tie
switch(es), and any DER whose support the plan credits:

$$
A^{\text{FLISR}}_k = \Pr\Big[\bigwedge_{d \in D(k)} \phi_d(X)\Big].
$$

Note the conjunction is over the **same** $X$ — shared cut sets (the feeder's
fiber ring, the DMS) correlate the terms, which is exactly the common-cause
effect Level 0 cannot see. Evaluate jointly (one BDD / one cut-set family per
contingency), never as $\prod_d A_d$ (⚠ that product over-counts shared
elements and *overestimates* failure probability of the chain — wrong sign of
error for redundancy decisions).

### 3.3 Interface to the physical engine: the interface matrix ✓

Partition the cyber-layer state space, per contingency $k$, into a small number
of **consequence-equivalent classes** $c \in \mathcal C(k)$ — states that lead
to the same restoration behaviour. The minimal useful partition is:

- $c_1$: full automation (FLISR + DER dispatch available),
- $c_2$: automation lost, manual restoration (crews), DER frozen,
- optionally $c_{1.5}$: partial (e.g. remote switching up, DER comms down).

with probabilities $\pi_c(k)$ computed from §3.2 ($\pi_{c_1} = A^{\text{FLISR}}_k$,
etc.). This is Lei–Singh's cyber-physical interface matrix specialized to
distribution restoration: the cyber evaluation ends at the vector
$\{\pi_c(k)\}$, and the physical consequence engine is invoked **once per
class** with a class-conditioned patch and class-conditioned durations. The two
layers stay solver-decoupled — no co-simulation needed below Level 3.

---

## 4. How cyber state enters the reliability mathematics

### 4.1 The staged-FMEA extension (the core formula) ✓

Baseline (implemented, `reliability_assessment_models.md` §5):

$$
\text{EENS} = \sum_k \lambda_k \big( S^{sw}_k \tau^{sw}_k + S^{rep}_k \tau^{rep}_k \big).
$$

Cyber-physical extension — condition every stage on the cyber class:

$$
\boxed{\;
\text{EENS} = \sum_k \lambda_k \sum_{c \in \mathcal C(k)} \pi_c(k)
\sum_{s \in \text{stages}} S_{k,c,s}\; \tau_{k,c,s}
\;}
$$

with, for the minimal two-class partition:

| | class $c_1$ (automation up), prob $A_k$ | class $c_2$ (automation down), prob $1-A_k$ |
|---|---|---|
| isolation/switching duration | $\tau^{a} \sim$ 1–5 min (FLISR) | $\tau^{m} \sim$ 45–120 min (crew travel + manual sectionalizing) |
| switching-stage patch | fault isolated, ties auto-closed, DER dispatched | fault isolated slowly, **no** tie reconfiguration until crew, DER `SetpointFrozen` |
| repair-stage shed $S^{rep}$ | with reconfiguration + DER | without DER re-dispatch (frozen), same faulted element out |

Interpretation: **cyber unavailability multiplies the *duration* of the
high-shed early stages and degrades the *consequence patch* of every stage.**
Both effects reuse existing machinery: durations are already per-mode fields
(`FailureModeReliability::isolation_hr/switching_hr`), and the degraded
patches are compositions of existing `MutationKind`s
(`RemoveControllability`, `RestorationActionForbidden`,
`RemoveGridForming`) via the shipped `compose_consequence_patches`.

The same conditioning applies verbatim to the three-stage MILP (stage
boundary times become class-dependent, Stage-2 switch budget $K^{sw}=0$ in
class $c_2$ until crew arrival) and to LOLE/LOLF/SAIFI/SAIDI (the IEEE-1366
accumulators just receive class-weighted CIF/CID contributions). ✓ This is a
conditional-expectation refinement — no new estimator theory, no new indices,
and it degrades gracefully to the baseline when $A_k \to 1$.

**Independence caveat ◐:** the formula assumes the cyber state at fault time is
independent of the fault itself. Weather common cause (a storm that fells
lines *and* wireless links) violates this; treat via §5.3/§7, not by
distorting $A_k$.

### 4.2 Channel-by-channel mapping

- **C1 Observability → isolation duration.** Fault-location time depends on
  fault-indicator/FTU coverage along the faulted section:
  $\tau^{iso}_k(o) = \tau^{loc}(o_k) + \tau^{op}$, where $o_k$ = fraction of
  monitored sectionalizing points on the path. A simple, defensible model:
  $\tau^{loc} = \tau^{loc}_{\text{auto}}$ if the bounding indicators report,
  else the crew-patrol time $\propto$ unmonitored section length.
  ◐ approximate but data-light; upgradeable later.
- **C2 Controllability → patches (shipped) + durations (new).** Frozen
  setpoints and lost grid-forming are exactly the F15 mutations; the new part
  is charging them for the correct *window* (class-conditioned stages) instead
  of the whole-mode duration.
- **C3 Protection.** Already representable: fail-to-trip is active-on-demand
  with $\nu_d$ = the frequency of primary faults in the zone (computable from
  the physical $\lambda$s — wire this instead of asking the user for $\nu_d$),
  $p_d$ = stuck probability; consequence = `ProtectionZoneExpansion`. Nuisance
  trip is a *new initiating event*: an additional passive mode on the breaker
  with its own $\lambda^{nt}$ whose consequence is a clean forced outage of a
  healthy element (cheap, high-value addition — comm-assisted protection
  mis-operation is a documented real-world contributor).
- **C4 Power supply of cyber nodes.** For single-fault adequacy this is second
  order ✓-negligible: the comm node is dead only if its feeding bus is already
  de-energized *longer than $T_b$*, which presupposes an ongoing outage; the
  product $U_{\text{bus}} \cdot U_{\text{extra}}$ is $\lesssim 10^{-6}$.
  **Do not build analytical machinery for it.** It matters in exactly two
  places: (a) sequential MC during multi-fault/storm periods (§5.3), where
  battery depletion is a simple timer on the simulated timeline; (b) the
  resilience track. Anything more is ⚠ effort misallocation.

### 4.3 What must stay out of EENS ✗

State-estimation corruption (`MeasurementBias`), false-data injection, and
optimality loss of DMS applications change *operating decisions*, not the
steady-state min-shed feasible set. The baseline's honest position — report
these `unsupported` by the adequacy engines — remains correct at every level
of this roadmap. Folding an assumed "% worse dispatch" into EENS would
manufacture precision. If quantifying them becomes a requirement, that is an
operational (time-series OPF / state-estimation-in-the-loop) study, not a
reliability index.

---

## 5. Method-by-method extension

### 5.1 Analytical: failure-mode FMEA and three-stage MILP (Levels 1–2)

Exactly §4.1. Per contingency $k$: evaluate $\{\pi_c(k)\}$ (Level 1: direct
per-feeder parameter; Level 2: cut sets on $G_c$), then run the existing
consequence engine once per class with the class patch/durations. Cost factor
≈ $|\mathcal C|$ (2–3×) on today's FMEA runtime — negligible. All existing
verdicts about engine fidelity (DC-PF hybrid LP, LinDistFlow MILP, F11/F12)
carry over unchanged, because the cyber layer only chooses *which* patch and
*for how long*.

### 5.2 Non-sequential MC (Level 2)

Extend the sampled state vector: $x = (x_p, x_c)$ with independent Bernoulli
components (cyber $U_c$ from the same resolver). Per sample: evaluate cyber
structure functions on $x_c$ (graph connectivity — cheap BFS), derive the
class, apply the class patch, evaluate shed. Two ⚠ cautions:

- NSQ has **no time axis**, so the duration channel must enter through
  class-conditioned *effective unavailability* of restoration
  (the $\tau$-weighting is implicit in $U$); document that NSQ-EENS remains
  "one load level, expected-state" and that the automation-value question is
  better answered by FMEA (explicit $\tau$) or SEQ.
- The baseline **F8** finding (NSQ tail risk invalid) becomes *more* misleading
  with cyber common-cause states in the mix; keep VaR/CVaR sequential-only.

### 5.3 Sequential MC (Level 3 — the reference method) ✓

The chronological simulator is where the genuinely dynamic couplings live, and
it needs *no new estimator theory* — only state on the timeline:

- per cyber component, alternate exponential up/down sojourns (identical
  sampling to physical, `run_sequential_mc` pattern);
- at each physical fault event, read the *current* cyber state → class →
  restoration timeline for that event (isolation at $t+\tau^{iso}_c$, ties at
  $t+\tau^{sw}_c$, repair at $t+\text{MTTR}$);
- C4: while a bus is de-energized, start battery timers on the cyber nodes it
  feeds; on expiry, mark them down until bus re-energization + reboot time
  (a few minutes–hours). This is the *only* level at which C4 is honestly
  representable;
- weather common cause: drive $\lambda(t)$ multipliers for overhead lines *and*
  wireless links from the same hazard time series (the typhoon/resilience
  scenario machinery already generates such series — reuse it as an optional
  hazard input);
- tail metrics (VaR/CVaR of annual EENS) remain valid here per the baseline
  F3/F8 analysis, and now include cyber-correlated bad years.

◐ Note the baseline's ≥1-hour event discretization (`max(1,·)` rounding) will
mask the FLISR-vs-manual difference (minutes vs. an hour); sub-hour event
timing inside the SEQ loop (or 1-minute buckets during events) is a required
minor rework for Level 3 to be meaningful.

### 5.4 Markov / state-space composition ◐ (verification only)

The joint chain over $(x_p, x_c)$ is exact but explodes as
$2^{n_p+n_c}$. Useful solely as a ≤6-component unit-test oracle for the
class probabilities and the staged formula (the same role the COPT/F&D method
plays for generation adequacy today). Do not productize.

### 5.5 Adversarial threats ✗ for reliability indices (Level 4)

Attacks are strategic, not stochastic: no $\lambda$ exists, and an intelligent
adversary targets precisely the cut sets §3.2 exposes. The honest treatments
are (a) **worst-case**: attacker–defender(–operator) interdiction — "which $k$
cyber elements, if compromised, maximize shed given optimal restoration"
(bilevel/tri-level MILP over the same LinDistFlow restoration kernel — the
reconfiguration doc's ONR with an adversarial upper level); or (b)
**scenario-conditional**: EENS *given* a postulated compromise scenario, using
the Level 2 machinery with $U\!=\!1$ on the compromised set. Both produce
resilience-style outputs (worst-case shed, N-k cyber survivability), reported
**beside**, never inside, EENS/SAIDI. This mirrors the repo's existing
physical-resilience separation and keeps both metric families meaningful.

---

## 6. Metrics: extensions, not replacements

Keep every existing index; add decompositions and three cyber-native indices:

- **EENS decomposition** (per contingency and system-level):
  $\text{EENS} = \text{EENS}^{\,\text{auto}} + \Delta^{\text{cyb-dur}} +
  \Delta^{\text{cyb-ctl}} + \Delta^{\text{prot-mis}}$ — the perfect-automation
  baseline, the slow-restoration increment, the frozen-DER increment, and the
  protection-misoperation increment. Computable exactly from the class-
  conditioned sum by toggling terms; this is the number that prices automation
  investment.
- **Bounding pair ✓ (cheap, decision-grade):** run FMEA once with all
  $A_k\!=\!1$ ("perfect cyber") and once with all $A_k\!=\!0$ ("no automation").
  The true system lives between; the gap is the total value at stake in the
  cyber layer, and requires *zero* cyber data. Recommended as the first
  deliverable of Level 1 — if the gap is small for a given case, stop there
  for that case.
- **Automation efficacy** $\eta = \dfrac{\text{EENS}^{A=0} - \text{EENS}}{\text{EENS}^{A=0} - \text{EENS}^{A=1}} \in [0,1]$ —
  how much of the achievable automation benefit current cyber reliability
  actually delivers. ◐ Note: with a *uniform* scalar availability the Level-1
  mixture is linear in $A$, so $\eta \equiv A$ by construction; the index
  becomes informative once per-component availabilities differ (overrides,
  Level 2 cut sets).
- **Expected observability/controllability loss**: $\sum_d w_d (1-A_d)$ —
  a pure cyber-layer KPI, reportable even with no physical run (GUI panel
  analog of the data-quality report).
- **Cyber-caused SAIDI share**: the share of SAIDI in excess of the
  perfect-cyber bound, $(\text{SAIDI}-\text{SAIDI}^{A=1})/\text{SAIDI}$ —
  "how much of today's SAIDI is attributable to imperfect automation".
  (An alternative reading — class-$c_2$-weighted CID over total CID — is
  auditable against utilities' "automation failed to operate" event logs; the
  implemented definition is the excess-share form. IEEE 1366 major-event
  discipline applies unchanged.)

Every reported figure carries the existing `model_scope` / `validity`-flag
discipline, extended with e.g. `cyber_topology_modelled`,
`restoration_duration_cyber_conditioned`, `cyber_power_coupling` (SEQ only).

---

## 7. Data model and parameters

New inputs, in ascending burden (all resolved via `resolve_reliability_params`
with the same provenance tags — `case|template|default|missing` — and the same
`StrictCaseDataOnly` honesty):

| Level | Data | Typical order of magnitude (template placeholders, ⚠ replace with utility/vendor data) |
|---|---|---|
| 1 | per-feeder (or per-device) automation availability $A$; manual vs auto switching times $\tau^m, \tau^a$ | $A \sim 0.9$–$0.99$; $\tau^a \sim 2$–5 min; $\tau^m \sim 1$–2 h |
| 2 | cyber node/link inventory + topology; per-kind $\lambda, r$: FTU/RTU, Ethernet switch/router, fiber segment, wireless BS, SCADA/DMS front-end (redundancy noted); service bindings $\beta$ | terminal units $\lambda \sim 0.01$–$0.1$/yr, $r \sim 2$–24 h; fiber $\lambda \sim 0.01$–$0.05$/yr·km, $r \sim 6$–24 h; central servers MTBF $10^4$–$10^5$ h with failover minutes |
| 2 (C3) | protection $p_d$ (fail-to-trip per demand), nuisance-trip $\lambda^{nt}$ | $p_d \sim 10^{-3}$–$10^{-2}$; $\lambda^{nt} \sim 0.01$–0.1/yr |
| 3 | power bindings $\pi$, battery ride-through $T_b$, reboot times; hazard→$\lambda(t)$ multipliers shared with lines | $T_b \sim 2$–8 h (substation), 0.5–4 h (pole-top) |

JSON: a sibling `cyber_system` block in the case schema — `nodes[]` (kind,
name, power_bus, battery_hr, λ/MTTR fields), `links[]` (from, to, kind, λ/MTTR),
`control_centers[]` (redundancy group), `bindings[]` (device stable_id →
terminal node). Component-kind enum gains `CommTerminal`, `CommLink`,
`CommSwitch`, `ControlCenter` — they slot into the existing
`ReliabilityComponentKind`/catalog machinery so coverage reporting,
templates, and strict mode work for cyber elements with zero new plumbing.

---

## 8. Code touch-point map (for effort estimation, not a work order)

| Piece | Kind of change | Reuses |
|---|---|---|
| `model/`: `CyberSystem` struct (+ case IO) | new, isolated | component IO mapping patterns |
| `reliability/cyber_function.cpp`: structure functions, min-cut $A_d$, class probabilities $\pi_c(k)$ | new, pure | resolver for parameters |
| failure-mode catalog: emit modes for cyber components; wire $\nu_d$ of fail-to-trip from zone $\lambda$s | extension | `build_failure_mode_catalog` |
| FMEA + failure-mode FMEA: class-conditioned stage loop (§4.1) | modify aggregation loop | consequence engines untouched |
| three-stage MILP: class-dependent stage times, $K^{sw}$ | small, **after F7 fix** | existing MILP |
| SEQ MC: cyber sojourn sampling, event-time class lookup, battery timers, sub-hour event timing | moderate | `run_sequential_mc` skeleton |
| NSQ MC: joint $(x_p,x_c)$ sampling + class patch | small | `sample_state`, dedup |
| metrics/GUI: EENS decomposition, bounding pair, new validity flags | small | IEEE-1366 + validity plumbing |
| out of scope: attack interdiction (resilience track), state-estimation-in-the-loop | — | — |

Untouched by design: the parameter resolver, the consequence engines
(DC-OPF / hybrid LP / LinDistFlow), `ConsequencePatch` machinery, IEEE-1366
math, tail-risk estimators. That containment is the main architectural payoff
of the interface-matrix boundary (§3.3).

---

## 9. Decision matrix and recommendation

| Level | Model | New data | New code | Answers that the previous level cannot | Verdict |
|---|---|---|---|---|---|
| **0** (today) | per-device cyber modes, no topology, no time channel | — | — | "does a frozen VSC shed load *now*" | shipped |
| **1** | scalar $A$ per feeder/device; class-conditioned staged FMEA (§4.1); bounding pair | 2–3 scalars per feeder | days–1 wk | **value of automation** in EENS/SAIDI; whether cyber matters *at all* for a given case | **✓ shipped** (FMEA method: `CyberPhysicalFMEAOptions`, GUI 网络物理 L1 panel, `cyber_physical_reliability_demo` case) |
| **2** | explicit $G_c$, cut-set $A_k$, common-cause via shared elements; joint NSQ; protection $\nu_d$ wiring | comm inventory + per-kind λ/r | 2–4 wk | *which* cyber element/redundancy dominates; correlated feeder-wide automation loss; N-1 cyber contingency ranking | **target architecture** |
| **3** | SEQ co-simulation: battery depletion, storm common cause, valid cyber tail risk | bindings, $T_b$, hazard series | 2–3 wk on top of 2 | storm-year tails; C4; multi-event interactions | build **only** via existing SEQ; needs sub-hour events |
| **4** | adversarial interdiction / scenario-conditional compromise | threat scenarios | separate track | worst-case N-k cyber survivability | keep **outside** reliability indices; resilience track |

**Recommendation.**

1. **Fix F7 and F10 first** — the cyber extension is duration- and
   parameter-borne; both findings corrupt exactly those channels.
2. **Build Level 1 immediately** and run the bounding pair
   ($A\!\equiv\!1$ vs $A\!\equiv\!0$) on the reference cases. This is days of
   work, needs no new data, and tells you per-case whether the cyber layer is
   worth modeling at depth — the cheapest possible de-risking of the whole
   roadmap. *(✓ done — shipped for the FMEA method with per-contingency
   bounding values, EENS decomposition, and the demo case; see the status
   header.)*
3. **Commit to Level 2 as the architecture** (cyber graph + interface matrix +
   catalog integration), because it is the lowest level at which the
   common-cause structure — the thing Level 0/1 provably understate — exists,
   and it is the natural substrate for Levels 3 and 4.
4. **Scope Level 3 to the existing sequential MC** (no external co-simulation
   framework; the exponential-sojourn skeleton already generalizes), and
   accept its sub-hour-event rework as part of that ticket.
5. **Refuse Level 4 inside EENS.** Offer it as a resilience-track study
   (interdiction over the restoration MILP) if and when required.

---

## 10. Canonical references (starting points, not a survey)

- R. Billinton, R. N. Allan, *Reliability Evaluation of Power Systems*, 2nd ed.
  — the two-state, F&D, and staged-restoration foundations the baseline
  already follows.
- IEEE Std 1366 — distribution index definitions (unchanged by this extension).
- B. Falahati, Y. Fu (et al.), *IEEE Trans. Smart Grid* (2012; 2014) — direct
  vs. indirect cyber-power interdependencies; the two-paper split maps exactly
  onto patches (C2/C3) vs. durations/functions (C1).
- H. Lei, C. Singh (et al.) — cyber-physical interface matrix; substation
  protection-system reliability under IEC 61850 — the class-probability
  boundary adopted in §3.3.
- C.-C. Liu, C.-W. Ten et al., *IEEE Trans. Power Systems* (2008) —
  SCADA vulnerability assessment via attack trees (Level 4 framing).
- J. Salmeron, K. Wood, R. Baldick, *IEEE Trans. Power Systems* (2004) —
  interdiction models for worst-case analysis (Level 4 machinery).
- IEC 61850 / IEEE 1547-2018 — communication dependence of protection schemes
  and DER control, i.e. why C2/C3 are not optional in modern feeders.

Cross-references: physical baseline and finding numbers —
[`reliability_assessment_models.md`](reliability_assessment_models.md);
restoration kernel — [`network_reconfiguration_models.md`](network_reconfiguration_models.md);
hazard scenario machinery — `src/scenario_generation/typhoon_resilience.cpp`.

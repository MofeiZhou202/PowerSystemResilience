# Reliability Mathematical Models and Intelligent Cyber-Physical Extension

> Verified against the public contracts, implementations, and reliability tests
> on 2026-07-19.
>
> Implementation sources: `include/hacdcpf/reliability/`,
> `include/hacdcpf/analysis/three_stage_reliability.hpp`, and
> `src/reliability/`. When this document and the code differ, the code and
> registered tests are authoritative.
>
> Status convention: **Implemented** means available in the current runtime;
> **Approximate** means implemented with the stated validity boundary;
> **Proposed** means a mathematical extension, not current runtime behavior.
> Sections labelled **Proposed** may reuse dynamics or short-circuit kernels
> that exist outside the reliability workflow. Reported EENS, LOLE, LOLF,
> SAIDI, and related metrics are protection/FRT-aware only when the result
> explicitly reports `protection_frt_reliability_coupled=true` and identifies
> the coupled event path.
>
> This consolidated export supersedes implementation-status claims in the older
> focused reliability documents where later source changes have fixed an issue.
> Those documents remain useful for historical rationale and focused derivation.

## 1. Purpose and scope

This document exports the mathematical models used by the reliability module
into one reference and develops a unified extension for intelligent
cyber-physical reliability assessment.

In the extension:

- **Intelligent** means fault detection, fault isolation, service restoration,
  and protection decision functions, including their uncertainty, latency, and
  failure modes.
- **Cyber** means information and communication infrastructure: sensors, IEDs,
  FTUs/RTUs, protection messages, communication links, gateways, SCADA/DMS,
  edge controllers, clocks, and control centers.
- **Physical** means the AC/DC electrical network, switches, protection zones,
  converters, microgrids, grid-forming and grid-following DERs, storage, and
  flexible demand.

The central modeling principle is:

$$
\text{reliability} = \text{event frequency} \times
\text{class-conditioned consequence} \times \text{duration}.
$$

The existing code already supplies the component reliability resolver, failure
mode catalog, consequence-patch mechanism, physical load-shed engines, staged
restoration models, and reliability metrics. The recommended extension adds a
cyber graph and explicit intelligent-function performance models at the point
where they change the physical consequence or restoration time.

## 2. Notation and index spaces

### 2.1 Sets and indices

| Symbol | Meaning |
|---|---|
| $i,j \in \mathcal B^{ac}$ | AC buses |
| $d,e \in \mathcal B^{dc}$ | DC buses |
| $\ell \in \mathcal L$ | AC or DC branches |
| $g \in \mathcal G$ | dispatchable sources |
| $r \in \mathcal R$ | DERs, including PV, storage, and converters |
| $m \in \mathcal M$ | failure modes |
| $k \in \mathcal K$ | initiating contingencies |
| $s \in \mathcal S$ | restoration stages |
| $c \in \mathcal C(k)$ | cyber/intelligent consequence classes for event $k$ |
| $u,v \in \mathcal V_c$ | cyber nodes |
| $a \in \mathcal A$ | intelligent decisions or restoration actions |
| $t$ | chronological time index |

The implementation distinguishes stable component IDs, vector positions, and
temporary graph indices. Exported results must use stable component identity.
AC and DC bus identifiers must remain domain-qualified even when their integer
values happen to be equal.

### 2.2 Reliability and consequence quantities

| Symbol | Unit | Meaning |
|---|---:|---|
| $\lambda_m$ | failure/up-year | annualized up-state failure intensity resolved for passive mode $m$ |
| $f_m^{cal}$ | event/year | calendar-exposure occurrence frequency used for annual consequence weighting |
| $\lambda_k$ | event/year | calendar-exposure frequency of initiating contingency $k$ |
| $\nu_m$ | demand/year | demand frequency for an active mode |
| $p_m^d$ | 1/demand | probability of active failure per demand |
| $r_m$ | hour | mean repair or cyber recovery duration |
| $U_m$ | dimensionless | steady-state unavailability |
| $H$ | hour/year | reporting horizon, normally 8760 or 8736 |
| $S(x)$ | MW | minimum total shed in state $x$ |
| $s_i(x)$ | MW | shed at bus or load point $i$ |
| $N_i$ | customer | customers represented at node $i$ |
| $\varepsilon$ | MW | threshold defining a loss-of-load state |

### 2.3 Reliability-index units

| Index | Unit | Interpretation |
|---|---:|---|
| EDNS | MW | expected demand not supplied at a random time |
| EENS | MWh/year | expected annual energy not supplied |
| PLC / LOLP | dimensionless | probability of load curtailment/loss |
| LOLE | hour/year | expected loss-of-load duration per year |
| LOLF | event/year | expected loss-of-load episode frequency |
| LOLD | hour/event | mean duration per loss-of-load episode |
| SAIFI | interruption/customer-year | sustained customer interruption frequency |
| SAIDI | hour/customer-year | sustained customer interruption duration |
| CAIDI | hour/interruption | mean duration per customer interruption |
| ASAI / ASUI | dimensionless | service availability/unavailability fraction |
| EENS VaR / CVaR | MWh/year | quantile/tail mean of annual EENS |

Every use of $\lambda$ or $F$ below states its exposure basis. Quantities with
the same numerical unit but different exposure bases are not interchangeable:
up-state failure intensity, calendar-year event frequency, failed-demand
frequency, nuisance-trip frequency, and common-cause group-event frequency are
distinct parameters.

## 3. Common reliability parameter model

### 3.1 Two-state repairable component

**Implemented.** `resolve_reliability_params` maps heterogeneous case fields to
$\{\lambda,r,U,\mathrm{MTTF}\}$. With annualized up-state failure intensity
$\lambda$, repair time $r$, and $H$ hours/year,

$$
\mu=\frac{H}{r}, \qquad
U=\frac{\lambda}{\lambda+\mu}
 =\frac{\lambda r}{\lambda r+H}.
$$

If MTTF and repair time are supplied,

$$
\lambda=\frac{H}{\mathrm{MTTF}}, \qquad
U=\frac{r}{\mathrm{MTTF}+r}.
$$

If forced-outage rate $f$ is supplied as steady-state unavailability,

$$
U=f, \qquad
\mathrm{MTTF}=\frac{r(1-f)}{f}, \qquad
\lambda=\frac{fH}{(1-f)r}.
$$

The legacy MTBF field is interpreted according to the declared convention:

$$
\mathrm{MTTF}=
\begin{cases}
\mathrm{MTBF}, & \text{MTBF means time to failure},\\
\max(\mathrm{MTBF}-\mathrm{MTTR},0), & \text{MTBF means full cycle time}.
\end{cases}
$$

The second branch above is the current compatibility behavior. The theoretical
identity is

$$
\mathrm{MTTF}=\mathrm{MTBF}_{cycle}-\mathrm{MTTR},
\qquad
\mathrm{MTBF}_{cycle}>\mathrm{MTTR}.
$$

If the validity condition fails, the data are inconsistent and a strict
scientific workflow should reject them. The implementation currently sanitizes
the result with $\max(\cdot,0)$; that is an input-compatibility rule, not a
statistical theorem, and must be reported when activated.

#### Theoretical assumptions and interpretation

The most general justification is an **alternating renewal process**. Let
$T^\uparrow$ and $T^\downarrow$ be independent, identically distributed
up-times and down-times with finite means. Assume:

- the component has exactly two mutually exclusive states, up and down;
- every failure is followed by repair, and repair returns it to an
  as-good-as-new state;
- successive up/down cycles are statistically identical and ergodic;
- failure and repair distributions are stationary over the study period;
- no aging, weather modulation, common-cause failure, repair-crew queue,
  partial-capacity state, or planned outage is embedded in this tuple.

The renewal-reward theorem gives the long-run unavailability

$$
U=\frac{\mathbb E[T^\downarrow]}
{\mathbb E[T^\uparrow]+\mathbb E[T^\downarrow]}
=\frac{\mathrm{MTTR}}{\mathrm{MTTF}+\mathrm{MTTR}}.
$$

This result does **not** require exponential holding times. Exponential
up/down times are additionally required when the model is interpreted as a
two-state continuous-time Markov chain. With hourly transition intensities

$$
\alpha=\frac{\lambda}{H}, \qquad \beta=\frac1r,
$$

the stationary equations

$$
\pi_\uparrow\alpha=\pi_\downarrow\beta,\qquad
\pi_\uparrow+\pi_\downarrow=1
$$

give

$$
U=\pi_\downarrow=\frac{\alpha}{\alpha+\beta}
=\frac{\lambda r}{H+\lambda r}.
$$

Here $\lambda$ must mean the failure intensity per **up-year** (the transition
hazard while the component is operational). If the input is instead an
empirical failure count per **calendar year**, denoted $f_{cal}$, renewal
reward gives

$$
U=\frac{f_{cal}r}{H}
$$

when $f_{cal}$ already counts completed failure cycles over calendar exposure.
The two definitions are related by

$$
f_{cal}=(1-U)\lambda.
$$

Consequently, for calendar-exposure observations,

$$
U_{cal}=\frac{f_{cal}r}{H}, \qquad 0\le f_{cal}r<H,
$$

and the equivalent up-state intensity is

$$
\lambda_{up}
=\frac{f_{cal}}{1-U_{cal}}
=\frac{f_{cal}}{1-f_{cal}r/H}.
$$

They are nearly equal only for small $U$. The current resolver interprets
failure-rate input as the up-state intensity $\lambda$. Studies using observed
calendar-year frequencies must convert the data or document the rare-event
approximation

$$
U\approx\frac{\lambda r}{H}, \qquad \lambda r\ll H.
$$

Multiple operating modes, partial derating, non-exponential aging, time-varying
weather hazards, dependent failures, and shared repair resources require a
multi-state, semi-Markov, or chronological model rather than this two-state
tuple.

### 3.2 Active-on-demand mode

**Implemented event-frequency reduction.** Protection, switching, and other
demanded functions use

$$
\lambda_m^{act}=\nu_m p_m^d.
$$

The dimensions are

$$
\left[\frac{\text{demands}}{\text{year}}\right]
\left[\frac{\text{failed operations}}{\text{demand}}\right]
=
\left[\frac{\text{failed operations}}{\text{year}}\right].
$$

Therefore $\lambda_m^{act}$ is an **expected failed-demand frequency**, not an
annual probability, not an MTTF by itself, and not the probability that the
function is unavailable when demanded.

The product is exact for the expected count when:

- $\nu_m$ is the expected number of genuine demands per year;
- $p_m^d$ is the conditional probability of failure for each such demand;
- the per-demand probability is constant, or $\nu_m p_m^d$ is replaced by the
  sum/integral of demand-specific probabilities;
- demand occurrence is independent of the latent function state and other
  modeled failures, unless that dependence is already included in $p_m^d$;
- one demand produces at most one counted failed operation.

If demands form a Poisson process and each demand independently fails with
probability $p_m^d$, Poisson thinning gives a failed-operation process with
rate $\nu_m p_m^d$. The probability of at least one failed operation in a year
is then

$$
\Pr(N_m^{fail}\ge1)=1-\exp(-\nu_m p_m^d),
$$

not $\nu_m p_m^d$ except as a rare-event approximation. With exactly $n$
independent demands,

$$
\Pr(N_m^{fail}\ge1)=1-(1-p_m^d)^n.
$$

For first-order FMEA, the expected consequence

$$
\mathrm{EENS}_m=\nu_m p_m^d\,\mathbb E[E_m\mid
\text{failed demand}]
$$

is appropriate when failed demands are rare and their recovery windows do not
materially overlap.

For heterogeneous initiating events, the more defensible form is

$$
\lambda_m^{act}=
\sum_{k\in\mathcal K_m}\lambda_k\,
\Pr(m\text{ fails}\mid k),
$$

or, with cyber/intelligent classes,

$$
\lambda_m^{act}=
\sum_{k\in\mathcal K_m}\lambda_k
\sum_c\pi_c(k)\Pr(m\text{ fails}\mid k,c).
$$

For protection fail-to-trip, $\mathcal K_m$ must contain the physical faults
that actually demand that relay/breaker. A generic demand-frequency default can
be useful for screening, but it is not a network-derived protection model.

If a shared driver $W$ changes both demand intensity and function success, the
correct marginal frequency is

$$
\lambda_m^{act}
=\sum_k\int
\lambda_k(w)\Pr(m\text{ fails}\mid k,w)f_W(w)\,dw.
$$

Replacing this integral by
$\mathbb E[\lambda_k(W)]\,\mathbb E[p_m^d(W)]$ would impose an independence
assumption and can miss precisely the severe weather/auxiliary-power states of
interest.

#### Demand failure versus time unavailability

For a dormant protection or switching function, the key quantity is often
probability of failure on demand, $\mathrm{PFD}_{avg}$, rather than ordinary
time unavailability. If dangerous undetected failures occur at rate
$\lambda_{DU}$ and perfect proof tests occur every $T_I$, a basic low-rate
approximation is

$$
\mathrm{PFD}_{avg}\approx\frac{\lambda_{DU}T_I}{2}.
$$

The failed-demand frequency is then

$$
\lambda_m^{act}=\nu_m\,\mathrm{PFD}_{avg},
$$

assuming demands are independent of the latent dangerous state. Diagnostic
coverage, imperfect proof testing, repair after detection, common-cause
failures, and demand/physical-fault dependence require a Markov, fault-tree, or
event-tree model.

When recovery duration $r_m$ is supplied, the current resolver additionally
computes

$$
U_m^{recovery}=
\frac{\lambda_m^{act}r_m}{H+\lambda_m^{act}r_m}.
$$

This can be interpreted only as the fraction of time spent in a post-failed-
demand recovery state under a Poisson/thinned two-state model. It is **not**
$p_m^d$ or $\mathrm{PFD}_{avg}$. Using it as the probability that a dormant
function fails on the next demand would be wrong.

Likewise, the current resolver value

$$
\mathrm{mttf\_hr}=\frac{H}{\lambda_m^{act}}
$$

is only a mean interval between failed-demand events under the Poisson-thinning
assumption. It must not be interpreted as the physical lifetime or latent
dangerous-failure MTTF of the demanded device.

The clean data contract should keep three quantities separate:

1. $p_m^d$ or $\mathrm{PFD}_{avg}$: conditional failure probability per demand;
2. $\lambda_m^{act}=\nu_m p_m^d$: expected failed operations per year;
3. $U_m^{recovery}$: time occupancy after a failed operation, when that metric
   is actually needed.

An active mode without demand frequency or probability per demand has no
resolved failed-demand frequency. A missing recovery time leaves recovery
occupancy and duration-weighted consequences unresolved, but it does not
invalidate the frequency $\nu_m p_m^d$ itself.

### 3.3 Data provenance

Every resolved parameter is tagged `case`, `template`, `default`, or `missing`.
The strict policy does not invent missing risk. The current Monte Carlo and FMEA
paths use the shared resolver and the same per-kind fallback table. This avoids
the historical inconsistency in which the same branch had different implied
unavailability in different methods.

## 4. Common consequence model

### 4.1 Failure-mode consequence operator

**Implemented.** A failure mode does not directly equal a component outage. It
is mapped to a network/control mutation through

$$
G_m=\Phi_m(G_0,x_m),
$$

where $G_0$ is the healthy rich system, $x_m$ is mode severity/state, and
$\Phi_m$ produces one or more mutations:

- topology: forced outage, stuck-open, or forced load interruption;
- capacity: rating derating by a residual-capacity factor;
- control: setpoint frozen, controllability removed, or grid-forming removed;
- protection: protection-zone expansion after fail-to-trip/fail-to-open;
- observation: measurement or communication degraded;
- restoration: an action is forbidden, such as fail-to-close.

Hard outage dominates derating and control loss on the same target. Incompatible
forced-open and forced-closed mutations are reported as conflicts. A mutation
that the selected physical engine cannot represent is marked unsupported rather
than assigned an artificial zero consequence.

### 4.2 Minimum-shed consequence

For a failed state $x$, each reliability method ultimately evaluates

$$
S(x)=\min_{y\in\mathcal F(G_x)} \sum_i w_i p_i^{sh},
$$

where $\mathcal F(G_x)$ is the feasible operating/restoration region of the
selected physical model. The implementation sets VOLL far above feasible
generation marginal costs so that source cost only breaks ties between
minimum-shed solutions. Reliability results read $S$ and $s_i$, not the
economic objective value.

### 4.3 Physical consequence-engine ladder

| Engine | Used by | Main physics | Validity boundary |
|---|---|---|---|
| AC DC-OPF | AC-only MC and FMEA | $P=B\theta$, nodal balance, source and branch limits, load shed | no voltage magnitude or reactive feasibility |
| Hybrid AC/DC network LP | hybrid MC, component FMEA, failure-mode FMEA | AC angle-flow equations; DC balances; bounded VSC/DC-DC transfer; storage/DER sources | DC transfer is linear and steady-state; no nonlinear AC validation |
| AC LinDistFlow restoration MILP | three-stage model | active/reactive balance, squared voltage, radial forest, switch decisions, storage | linearized losses; AC formulation |
| DC LinDistFlow LP | three-stage hybrid path | resistive voltage drop, DC voltage and line limits, bounded VSC support | falls back to capacity/connectivity if solve fails |

All public result objects expose `model_scope`, `model_limitations`, and/or
validity flags. These declarations are part of the mathematical result and must
be retained by downstream APIs.

## 5. Non-sequential Monte Carlo model

### 5.1 State sampling

**Implemented.** Each active component is sampled independently:

$$
X_m\sim\mathrm{Bernoulli}(U_m), \qquad
x=(X_1,\ldots,X_M).
$$

Thus the implemented joint state probability is

$$
\Pr(X=x)=\prod_{m=1}^{M}
U_m^{x_m}(1-U_m)^{1-x_m}.
$$

This factorization assumes steady-state random-time sampling, mutually
independent component states, no common cause, no repair-resource coupling,
and a consequence function that does not depend on outage history. An explicit
joint distribution or shared latent hazard is required when those assumptions
do not hold.

Base-case out-of-service elements are treated as baseline topology rather than
random failures. Under strict data policy, a component with missing reliability
data is not assigned an invented failure probability.

For $n$ sampled states, with $S_q=S(x_q)$,

$$
\widehat{\mathrm{EDNS}}=\frac1n\sum_{q=1}^{n}S_q,
$$

$$
\widehat{\mathrm{EENS}}=H\widehat{\mathrm{EDNS}}, \qquad
\widehat{\mathrm{PLC}}=\frac1n\sum_q\mathbf 1[S_q>\varepsilon],
$$

$$
\widehat{\mathrm{LOLE}}=H\widehat{\mathrm{PLC}}.
$$

The annualization is unconditional only if a sample also represents the annual
distribution of load, renewable availability, and operating regime. With a
fixed stress/load level $\rho^\star$ it is instead a conditional scenario
quantity:

$$
\mathrm{EENS}(\rho^\star)
=H\,\mathbb E_X[S(X,\rho^\star)].
$$

The general annual expectation is

$$
\mathrm{EENS}
=\sum_{h=1}^{H}\mathbb E[S(X_h,\rho_h)]\Delta t.
$$

Only when $(X,\rho)$ is sampled from the annual random-hour distribution may
this be reduced to $H\mathbb E_{X,\rho}[S(X,\rho)]$.

The result also separates unavoidable N-0 curtailment from outage-induced
increment:

$$
S_q^{inc}=\max(0,S_q-S_0), \qquad
\mathrm{EENS}^{inc}=H\,\mathbb E[S^{inc}].
$$

### 5.2 Convergence

With sample mean $\bar S$ and sample standard deviation $\sigma_S$, the stopping
diagnostic is the coefficient of variation of the estimated mean:

$$
\mathrm{CoV}(\bar S)=\frac{\sigma_S}{\bar S\sqrt n}.
$$

The run stops after minimum sampling requirements when this quantity is below
the configured threshold, or at the maximum iteration count.

This diagnostic is defined only for $\bar S>0$. The current implementation
sets its recorded CoV to zero when no incremental shed has been observed, but
also requires CoV to be positive before declaring convergence; therefore a
zero-loss run continues to the iteration cap. The recorded zero must not be
interpreted as statistical proof of convergence.

If no loss is observed in $n$ independent trials, a one-sided
$1-\alpha$ binomial upper bound for PLC is

$$
p_{upper}=1-\alpha^{1/n}.
$$

This bound, a Wilson/exact interval, or rare-event sampling is required to make
a defensible statement about a very reliable system.

### 5.3 Tail risk

**Implemented with approximation.** A non-sequential sample represents a random
hour, not a random annual total. Current code constructs synthetic years by
resampling hourly DNS values and summing $H$ draws:

$$
E_y=\sum_{h=1}^{H}S_{y,h}\Delta t.
$$

VaR and CVaR are then calculated over $\{E_y\}$. This fixes the invalid practice
of annualizing one sampled hour. It remains an iid-hour approximation and does
not reproduce weather persistence, repair chronology, storage depletion, or
load autocorrelation. Sequential Monte Carlo is the reference method for annual
tail risk.

For iid sampled hours with autocovariance $\gamma_\ell$ forced to zero for
$\ell>0$,

$$
\mathbb E[E_y]=H\mathbb E[S]\Delta t,\qquad
\mathrm{Var}(E_y)=H\gamma_0(\Delta t)^2.
$$

The true chronological variance is

$$
\mathrm{Var}(E_y)
=(\Delta t)^2\left[
H\gamma_0+2\sum_{\ell=1}^{H-1}(H-\ell)\gamma_\ell
\right].
$$

Positive persistence from weather, repair, load, and storage normally makes
the iid bootstrap understate dispersion and can materially distort VaR/CVaR.

### 5.4 Scope

The method is rigorous for independent two-state components at one sampled load
level and the stated consequence engine. It does not produce chronological loss
frequency. Component `importance` is a co-occurrence attribution, not a
Birnbaum derivative; the same multi-failure shed can be associated with several
down components.

## 6. Sequential Monte Carlo model

### 6.1 Alternating renewal process

**Implemented Markov specialization.** Each component alternates between
exponential up and down sojourns. With
$u_1,u_2\sim\mathcal U(0,1)$,

$$
T_m^{up}=-\mathrm{MTTF}_m\ln u_1, \qquad
T_m^{down}=-\mathrm{MTTR}_m\ln u_2.
$$

This is a continuous-time Markov model and is not a general alternating-renewal
sampler. Weibull aging, deterministic inspection, lognormal repair, and
weather-dependent holding times require distribution-specific inverse CDFs or
an event hazard model.

At hour $h$, the physical state and the chronological load profile determine

$$
S_h=S\!\left(x_h,\rho_h\right),
$$

where $\rho_h$ combines the common temporal profile, optional spatial load
factors, and the study stress multiplier.

### 6.2 Annual indices

For simulated year $y$,

$$
\mathrm{EENS}_y=\sum_{h=1}^{H} S_{y,h}\Delta t,
$$

$$
\mathrm{LOLE}_y=\sum_{h=1}^{H}\mathbf 1[S_{y,h}>\varepsilon]\Delta t,
$$

$$
\mathrm{LOLF}_y=\sum_{h=2}^{H}
\mathbf 1[S_{y,h}>\varepsilon,\ S_{y,h-1}\le\varepsilon].
$$

Reported metrics are means over simulated years. Annual VaR/CVaR is valid here
because each sample is a complete chronological year.

### 6.3 Scope

This is the current reference adequacy estimator for chronology and annual tail
risk. The time step is one hour and sampled durations are rounded to at least
one hour. That resolution is too coarse to distinguish a two-minute automatic
FLISR action from a twenty-minute action; sub-hour event scheduling is required
for the proposed intelligent extension.

The implementation rounds up-times to the nearest hour and down-times upward,
with a one-hour minimum. For a true constant-shed event of duration
$0<\tau<1$ hour,

$$
E=S\tau,\qquad E^{grid}=S\cdot1\text{ hour},
$$

so the relative energy/duration bias is

$$
\frac{E^{grid}-E}{E}=\frac{1-\tau}{\tau}.
$$

At $\tau=5/60$ hour this is 11, or a 1100% overestimate. This discretization is
minor for long repairs but unacceptable for fast FDIR/protection studies.

## 7. Frequency-and-duration COPT model

**Implemented.** This is a generation-adequacy model over generator capacity
outages. Let $O$ be total unavailable generation capacity and define

$$
P(X)=\Pr(O\ge X),
$$

and let $F(X)$ be the upward threshold-crossing frequency of the set
$\{O\ge X\}$, in crossings/hour. For each generator, the implementation derives
hourly transition intensities from FOR $q$ and MTTR $r$:

$$
q=\frac{\alpha}{\alpha+\beta},\qquad
\beta=\frac1r,\qquad
\alpha=\frac{q}{1-q}\beta
=\frac{q}{(1-q)r},
$$

where $q$ is steady-state FOR, $0\le q<1$, and $\alpha$ and $\beta$ have units
1/hour. For a new unit with capacity $C$, availability $p=1-q$, and failure
intensity $\alpha$, the recursive cumulative outage table is

$$
P_{new}(X)=pP(X)+qP(X-C),
$$

$$
F_{new}(X)=pF(X)+qF(X-C)+\alpha p\,[P(X-C)-P(X)].
$$

For reserve $R=C_{installed}-L_{peak}$,

$$
\mathrm{LOLP}=P(X>R), \qquad
\mathrm{LOLF}=H\,F(X>R),
$$

$$
\mathrm{LOLE}=H\,\mathrm{LOLP}, \qquad
\mathrm{LOLD}=\frac{\mathrm{LOLE}}{\mathrm{LOLF}}.
$$

The units are therefore

$$
[F]=\frac{\text{crossing}}{\text{hour}},\qquad
[HF]=\frac{\text{event}}{\text{year}}.
$$

The multiplication by $H$ is correct for the current source because the
recursion uses $\alpha$ in 1/hour. If a future implementation supplies failure
frequency directly in event/year, then the recursion produces event/year and
must use $\mathrm{LOLF}=F(X>R)$ without another factor of $H$.

The code stores the grid $\mathcal X=\{j\Delta C\}$ with $\Delta C=10$ MW. For
nonnegative reserve it evaluates the first grid threshold strictly above $R$:

$$
X_R=\min\{X\in\mathcal X:X>R\}
=\Delta C\left(\left\lfloor\frac{R}{\Delta C}\right\rfloor+1\right),
$$

and uses $P(O\ge X_R)$ and $F(O\ge X_R)$. Thus $X_R=R+\Delta C$ only when $R$
is grid-aligned. Each generator capacity is also represented by
$\lfloor C/\Delta C\rfloor$ grid steps. This precise cumulative definition is
required; the displayed recursion would not be valid if $P(X)$ were interpreted
as exact probability mass $\Pr(O=X)$. Studies with negative reserve or material
sub-grid units require an explicit boundary treatment or a finer grid rather
than relying on the current index clamp.

This is HL-I screening: it has no transmission/distribution network, voltage,
restoration, microgrid islanding, or load-duration curve. Capacity is discretized
on the implementation grid, so the reserve threshold has grid coarseness.

## 8. Deterministic component FMEA

### 8.1 Two-stage physical model

**Implemented.** Each in-service component $k$ is evaluated as an N-1 event.
The event has a switching/isolation stage followed by the remaining repair
window:

$$
\tau_k^{rep}=\max(0,r_k-\tau_k^{sw}).
$$

The fault remains out throughout both windows. The repair-stage engine may use
crew/switch reconfiguration, DER redispatch, storage, grid-forming VSC support,
black-start storage, and microgrid islanding according to options and device
capabilities. Storage support is power- and energy-limited:

$$
P_b^{avail}=\min\left(P_b^{rated},
\frac{(E_b-E_b^{min})\eta_b^{dis}}{\tau_s}\right).
$$

Grid-forming VSC and microgrid island anchors are capped at their declared
rating; they are not modeled as unlimited slack sources.

The exposure-consistent analytical aggregation is

$$
\mathrm{EENS}=\sum_k f_k^{cal}
\left(S_k^{sw}\tau_k^{sw}+S_k^{rep}\tau_k^{rep}\right),
$$

$$
\mathrm{LOLE}=\sum_k f_k^{cal}\left(
\tau_k^{sw}\mathbf 1[S_k^{sw}>\varepsilon]+
\tau_k^{rep}\mathbf 1[S_k^{rep}>\varepsilon]\right),
$$

$$
\mathrm{LOLF}=\sum_k f_k^{cal}
\mathbf 1[S_k^{sw}>\varepsilon\ \lor\ S_k^{rep}>\varepsilon].
$$

Under the two-state model,

$$
f_k^{cal}=(1-U_k)\lambda_k^{up}.
$$

The current implementation instead substitutes the resolver's numerical
$\lambda_k^{up}$ directly for $f_k^{cal}$. This is the rare-event approximation
$f_k^{cal}\approx\lambda_k^{up}$ and overweights the calendar event count by the
factor $1/(1-U_k)$ when interpreted literally. The difference is negligible
only for small $U_k$; long-repair, disaster, or highly unavailable components
require explicit calendar/up-state conversion.

### 8.2 Restoration search

The component FMEA repair-stage switching search is explicit but heuristic. It
enumerates bounded combinations of eligible AC switching actions and retains the
least-shed result. If the configured OPF-call budget is exhausted, the result is
flagged `repair_search_truncated`. DC-side switch reconfiguration is not
optimized by this search.

### 8.3 Validity

The formula is the standard first-order rare-event approximation. It is best for
non-overlapping N-1 events. It does not model the chronological interaction of
overlapping repairs, weather-correlated failures, storage use across events, or
crew competition.

The neglected overlap terms are second order, approximately

$$
\Pr(i,j\text{ simultaneously down})
=O\left(\frac{f_i^{cal} r_i}{H}
\frac{f_j^{cal} r_j}{H}\right).
$$

The implemented LOLF counts initiating contingencies that cause any loss, at
most one loss occurrence per contingency. It does not count multiple distinct
loss episodes within one event, such as loss, restoration, and later loss after
storage depletion.

## 9. Failure-mode FMEA and co-failures

### 9.1 Failure-mode catalog

**Implemented.** Rich components expand into passive and active failure modes
with cause classes: physical, cyber control, communication, measurement,
protection logic, human operation, and scheduled. Consequences include forced
outage, derating, stuck states, fail-to-open/close/trip, nuisance trip, frozen
setpoint, communication loss, loss of grid forming, and zone trip.

For a supported mode $m$, the current deterministic failure-mode result uses

$$
f_m=
\begin{cases}
\lambda_m, & \text{passive (implemented rare-event exposure approximation)},\\
\nu_m p_m^d, & \text{active-on-demand},
\end{cases}
$$

$$
d_m=\tau_m^{iso}+\tau_m^{sw}+
\max(r_m,r_m^{mode}),
$$

$$
\mathrm{EENS}_m=f_m d_m S_m, \qquad
\mathrm{LOLE}_m=f_m d_m\mathbf 1[S_m>\varepsilon].
$$

For an exposure-consistent passive mode, use

$$
f_m=f_m^{cal}=(1-U_m)\lambda_m^{up}
$$

instead of the implemented direct substitution $f_m=\lambda_m^{up}$. The
active-on-demand branch already has event/year units and does not receive this
factor; applying $(1-U)$ to $\nu p^d$ would conflate post-event recovery
occupancy with probability of failure on demand.

The current implementation therefore interprets the selected
$\max(r_m,r_m^{mode})$ as a repair/recovery duration that begins **after**
isolation and switching. This is valid only when the input contract has that
meaning. If a supplied cyber recovery or outage duration is measured from the
initiating event, adding $\tau_m^{iso}+\tau_m^{sw}$ double-counts time.

A strict schema must distinguish:

$$
d_m=
\begin{cases}
\tau_m^{iso}+\tau_m^{sw}+r_m^{post},&
r_m^{post}\text{ begins after switching},\\
\max(r_m^{total},\tau_m^{iso}+\tau_m^{sw}),&
r_m^{total}\text{ is total event-to-recovery time}.
\end{cases}
$$

Until that semantic is explicit in input data, deterministic failure-mode
duration is an approximation and should not be mixed with staged FMEA as if the
two models used identical clocks.

This model evaluates one steady consequence for the whole mode duration. It is
therefore less temporally detailed than staged component FMEA. Pure observation
loss, switch communication loss without a simultaneous physical fault, and
fail-to-close are honestly unsupported by this steady-state engine because their
consequence appears through restoration behavior rather than an immediate
single-mode shed state.

### 9.2 Second-order pair model

**Implemented as an optional approximation.** For distinct supported modes
$i,j$,

$$
U_i\approx\min\left(1,\frac{f_i d_i}{H}\right), \qquad
U_{ij}=U_iU_j,
$$

$$
d_{ij}=\frac{d_i d_j}{d_i+d_j}, \qquad
f_{ij}=\frac{f_i f_j(d_i+d_j)}{H},
$$

$$
\mathrm{EENS}_{ij}=U_{ij}H S_{ij}, \qquad
\mathrm{LOLE}_{ij}=U_{ij}H\mathbf 1[S_{ij}>\varepsilon].
$$

Pairs below the joint-unavailability threshold or beyond the evaluation cap are
skipped. Because the implementation adds the full pair consequence to the
first-order totals, this should be treated as an N-2 exposure/ranking
approximation, not an exact disjoint-state expansion. Exact second-order totals
would require mutually exclusive state probabilities or an interaction
correction such as $S_{ij}-S_i-S_j+S_0$.

Here $d_{ij}$ is an **equivalent overlap duration** chosen so that

$$
\frac{f_{ij}d_{ij}}{H}=U_iU_j.
$$

It is not the physical repair time of either component and is not a general
formula for mean overlap under arbitrary duration distributions. Its use
requires independent rare events and the matching frequency approximation.

The exact alternative is not implemented. A formal second-order interaction
expansion would use

$$
\mathbb E[S]\approx
S_0+\sum_iU_i(S_i-S_0)
+\sum_{i<j}U_iU_j(S_{ij}-S_i-S_j+S_0),
$$

with a consistent truncation of state probabilities. Consequently, current
pair contributions must be reported separately from the primary EENS total for
scientific interpretation, even though the present result object adds them.
All terms of order $O(U_iU_jU_k)$ and higher, plus repair-resource and
chronological overlap interactions, remain omitted.

## 10. Three-stage restoration reliability model

### 10.1 Stage semantics

**Implemented.** For contingency $k$:

| Stage | Interval | Physical meaning | Topology |
|---|---|---|---|
| 1 | $[0,\tau_{SW}]$ | fault clearing and isolation | fault open; ties nominal |
| 2 | $[\tau_{SW},\tau_{TP}]$ | switching restoration | fault open; eligible ties optimized |
| 3 | $[\tau_{TP},\tau_{RP}]$ | repair window | fault still open; accepted Stage-2 plan held |

The component is repaired only at $\tau_{RP}$, a zero-duration boundary. Stage 3
does not restore the faulted component. This current behavior is regression
tested for a radial load isolated throughout MTTR.

Durations satisfy

$$
r_k=\tau_k^{iso}+\tau_k^{sw}+\tau_k^{rep}.
$$

To avoid mixing absolute event times with stage durations, define

$$
t_{k,0}=0,\quad
t_{k,1}=\tau_{SW,k},\quad
t_{k,2}=\tau_{TP,k},\quad
t_{k,3}=\tau_{RP,k},
$$

and

$$
\Delta t_{k,1}=t_{k,1},\qquad
\Delta t_{k,2}=t_{k,2}-t_{k,1},\qquad
\Delta t_{k,3}=t_{k,3}-t_{k,2}.
$$

The implementation fields correspond to

$$
\tau_k^{iso}=\Delta t_{k,1},\quad
\tau_k^{sw}=\Delta t_{k,2},\quad
\tau_k^{rep}=\Delta t_{k,3},\quad
r_k=t_{k,3}.
$$

### 10.2 AC LinDistFlow restoration MILP

For each stage, the model minimizes active load shed:

$$
\min \sum_{i\in\mathcal B^{ac}} w_i p_i^{sh}+10^{-8}\sum_b p_b^{st}.
$$

The storage term only breaks dispatch degeneracy and preserves energy when
non-storage supply can serve the same load.

Energization and pickup:

$$
P_i^d-p_i^{sh}\le P_i^d y_i,
\qquad y_i\in\{0,1\},
$$

with source buses rooted as energized. Grid-following generation and storage
cannot energize a dead island:

$$
0\le p_i^g\le \overline P_i^g y_i, \qquad
0\le p_b^{st}\le \overline P_b^{st} y_{i(b)}.
$$

Active and reactive balance:

$$
\sum_{j:(j,i)}P_{ji}-\sum_{j:(i,j)}P_{ij}
+p_i^g+p_i^{sh}+\sum_{b:i(b)=i}p_b^{st}=P_i^d,
$$

$$
\sum_{j:(j,i)}Q_{ji}-\sum_{j:(i,j)}Q_{ij}
+q_i^g+q_i^{sh}=Q_i^d,
\qquad q_i^{sh}=\frac{Q_i^d}{P_i^d}p_i^{sh}.
$$

Measured `q_mvar`/`qd_mvar` is used when available; only missing/zero reactive
demand is reconstructed with a 0.9 power factor.

For squared voltage $v_i$ and branch state $z_{ij}$,

$$
-M(1-z_{ij})\le
v_j-v_i+\frac{2r_{ij}}{S_{base}}P_{ij}
+\frac{2x_{ij}}{S_{base}}Q_{ij}
\le M(1-z_{ij}),
$$

The unit convention is: $v_i$ is squared per-unit voltage; $r_{ij},x_{ij}$ are
per-unit impedance on system base; $P_{ij},Q_{ij}$ are MW/Mvar; and
$S_{base}$ is MVA. Therefore $P_{ij}/S_{base}$ and $Q_{ij}/S_{base}$ are
per-unit power. If physical ohms were supplied instead, the equation would
require voltage-base conversion and this form would be invalid.

$$
\underline V_i^2\le v_i\le\overline V_i^2,
$$

$$
-z_{ij}\overline S_{ij}\le P_{ij},Q_{ij}
\le z_{ij}\overline S_{ij}.
$$

The implementation uses separate linear bounds on $P$ and $Q$:

$$
|P_{ij}|\le z_{ij}\overline S_{ij},\qquad
|Q_{ij}|\le z_{ij}\overline S_{ij}.
$$

This box is an **outer, optimistic relaxation** of
$P_{ij}^2+Q_{ij}^2\le z_{ij}\overline S_{ij}^2$. It permits the corner
$P=Q=\overline S$, whose apparent power is $\sqrt2\,\overline S$. A
decision-grade model should use an SOCP constraint or polygonal inner
approximation such as

$$
P_{ij}\cos\theta_j+Q_{ij}\sin\theta_j
\le z_{ij}\overline S_{ij}\cos(\pi/J),\qquad j=1,\ldots,J.
$$

### 10.3 Strict radial energized forest

An auxiliary commodity $f_{ij}$ establishes connectivity from voltage-forming
roots:

$$
\sum f_{in}-\sum f_{out}=y_i, \quad i\notin\mathcal S_{root},
$$

$$
\sum f_{in}-\sum f_{out}\le0, \quad i\in\mathcal S_{root},
$$

$$
|f_{ij}|\le(|\mathcal B|-1)z_{ij}, \qquad
z_{ij}\le y_i,\quad z_{ij}\le y_j,
$$

$$
\sum_{(i,j)}z_{ij}\le\sum_i y_i-|\mathcal S_{root}|.
$$

The failed element is fixed open in every stage. Normally open restoration ties
are fixed open in Stage 1 and optimized in Stage 2. The accepted plan is held in
Stage 3. The switching budget is

$$
\sum_{\ell\in\mathcal L^{NO}}z_\ell\le K^{sw}.
$$

Protection/interlock prechecks verify that protection devices can interrupt
fault current and that sectionalizing actions occur only after the required
upstream trip. Invalid action sequences block restoration admission and are
reported.

### 10.4 Storage chronology inside an event

For stage duration $\Delta t_s$,

$$
E_{b,s+1}=E_{b,s}-p_{b,s}^{st}\Delta t_s,
$$

with discharge bounded by rating and remaining deliverable energy. Stage 3 is
re-solved only when needed to enforce changed storage-energy limits; otherwise
the exact accepted Stage-2 topology/solution can be reused because physical
topology and demand are unchanged.

### 10.5 DC LinDistFlow coupling

The default hybrid path solves a DC LP with

$$
v_e=v_d-2r_{de}P_{de},
$$

$$
\underline V_d^2\le v_d\le\overline V_d^2,
\qquad |P_{de}|\le\overline P_{de},
$$

and per-bus balance including DC sources, shed, bounded DC/DC transfers, and VSC
injections. VSC transfer is capped by converter rating and available AC-side
surplus. If the DC LP cannot be solved, a connectivity/capacity fallback is used
and the scope flags disclose that fallback.

### 10.6 Frequency weighting and N-0 qualification

Let $s_{i,k,s}^{raw}$ be shed under contingency $k$ and
$s_{i,s}^{N0}$ the healthy shed under matching stage duration/energy conditions.
The reliability increment is

$$
s_{i,k,s}^{inc}=\max(0,s_{i,k,s}^{raw}-s_{i,s}^{N0}).
$$

Then

$$
\mathrm{EENS}_i=\sum_k\lambda_k\sum_s
s_{i,k,s}^{inc}\tau_{k,s},
$$

$$
\mathrm{LOLE}=\sum_k\lambda_k\sum_s
\tau_{k,s}\mathbf 1\!\left[\sum_i s_{i,k,s}^{inc}>\varepsilon\right].
$$

The implemented N-0 counterfactual starts from the same initial storage energy
and uses the same three stage durations, but it evolves a separate healthy
dispatch/energy trajectory:

$$
s_{i,k,s}^{inc}
=\max\left(
0,\,
s_i(G_{k,s},E_{k,s})
-s_i(G_{0,s},E_{0|k,s})
\right).
$$

Here $E_{0|k,s}$ is the energy state reached by the healthy counterfactual under
the same exogenous duration sequence, not the contingency's depleted state.
This estimates the total causal effect of the fault, including its effect on
storage use. It is not a pointwise topology-only marginal. A topology-only
comparison would instead hold the same energy state in both solves. The
counterfactual remains valid only when both paths share initial SOC, load,
renewable availability, stage durations, and all non-fault study settings.

By default the fault set includes AC and DC branches. Generator, transformer,
converter, switch/breaker, and DER/microgrid fault families are option-controlled
so historical branch-only results remain reproducible.

## 11. Reliability indices and risk summaries

### 11.1 System and nodal indices

The common definitions are

$$
\mathrm{EDNS}=\mathbb E[S], \qquad
\mathrm{EENS}=H\,\mathrm{EDNS},
$$

$$
\mathrm{PLC}=\Pr(S>\varepsilon), \qquad
\mathrm{LOLE}=H\,\mathrm{PLC}
$$

for a state-sampling model, with method-specific chronological or staged
definitions used elsewhere. Nodal EENS retains the public layout
`[AC buses | DC buses]` for hybrid consequence engines.

### 11.2 IEEE 1366 customer indices

**Implemented.** With customer interruption frequency $\mathrm{CIF}_i$,
duration $\mathrm{CID}_i$, and customers $N_i$,

$$
\mathrm{SAIFI}=\frac{\sum_iN_i\mathrm{CIF}_i}{\sum_iN_i},
$$

$$
\mathrm{SAIDI}=\frac{\sum_iN_i\mathrm{CID}_i}{\sum_iN_i},
$$

$$
\mathrm{CAIDI}=\frac{\mathrm{SAIDI}}{\mathrm{SAIFI}}, \qquad
\mathrm{ASAI}=1-\frac{\mathrm{SAIDI}}{H}, \qquad
\mathrm{ASUI}=1-\mathrm{ASAI}.
$$

AC and DC customers are included. If explicit customer counts are missing, the
implementation uses approximately 10 customers/MW. Such results are
load-weighted proxies and must not be presented as measured customer indices.

These formulas follow the IEEE 1366 index form, but the current module does not
claim full IEEE 1366 event-processing compliance: major-event-day exclusion,
sustained-versus-momentary interruption classification, and all data-quality
rules are not implemented here.

### 11.3 VaR and CVaR

For annual loss random variable $E$ and confidence $\alpha$,

$$
\mathrm{VaR}_\alpha(E)=\inf\{e:\Pr(E\le e)\ge\alpha\}.
$$

The mathematically strict Expected Shortfall definition is

$$
\mathrm{ES}_\alpha(E)
=\frac{1}{1-\alpha}\int_\alpha^1\mathrm{VaR}_u(E)\,du.
$$

Equivalently, the Rockafellar-Uryasev representation is

$$
\mathrm{CVaR}_\alpha(E)
=\min_{\eta\in\mathbb R}
\left\{
\eta+\frac{1}{1-\alpha}\mathbb E[(E-\eta)^+]
\right\}.
$$

For a continuous distribution this equals
$\mathbb E[E\mid E\ge\mathrm{VaR}_\alpha(E)]$. For discrete or empirical
distributions, simply averaging every observation at or above the empirical
VaR can overweight a probability mass at VaR and is not generally equal to
Expected Shortfall.

The current implementation uses that simple tail-mean estimator beginning at
the empirical VaR index. It is an engineering approximation, especially for
small samples or many tied annual losses. Sequential annual samples are the
preferred input; non-sequential synthetic years remain the iid approximation
described in Section 5.3.

### 11.4 Sampling uncertainty and rare events

For independent state samples,

$$
\mathrm{SE}(\widehat{\mathrm{EDNS}})
=\frac{\widehat\sigma_S}{\sqrt n},\qquad
\mathrm{SE}(\widehat{\mathrm{EENS}})
=H\frac{\widehat\sigma_S}{\sqrt n}.
$$

An asymptotic $1-\alpha$ interval is

$$
\bar S\pm z_{1-\alpha/2}
\frac{\widehat\sigma_S}{\sqrt n}.
$$

For $\widehat p=\widehat{\mathrm{PLC}}$,

$$
\mathrm{SE}(\widehat p)
=\sqrt{\frac{\widehat p(1-\widehat p)}{n}},
$$

but Wilson or exact binomial intervals are preferred for rare loss. Annual
sequential samples require year-level variance and confidence intervals rather
than treating their hours as independent.

Plain Monte Carlo may be statistically ineffective for very small PLC/LOLE.
Importance sampling, cross-entropy adaptation, subset simulation, contingency-
order stratification, or minimal-cut-set-guided sampling are proposed remedies.
The public importance-sampling option is currently experimental and must not be
claimed effective without evidence that likelihood-ratio weighting is applied.

## 12. Implemented Level-1 cyber-physical FMEA

### 12.1 Interface classes

**Implemented for deterministic component FMEA.** For contingency $k$, the
current model uses two consequence-equivalent cyber classes:

- $c=a$: automation available, probability $A_k$;
- $c=m$: automation unavailable, manual response, probability $1-A_k$.

The probability is contingency-conditioned:

$$
A_k=\Pr(C=a\mid K=k),\qquad
1-A_k=\Pr(C=m\mid K=k).
$$

The current global scalar or per-component override is a screening input. It
implicitly assumes either physical/cyber independence or that the user has
already supplied the conditional value. In general
$\Pr(C=a\mid K=k)\ne\Pr(C=a)$ because weather, auxiliary power, site damage,
and shared infrastructure can drive both layers.

Availability can be global or overridden per component. Automatic switching
time is $\tau_k^a$; manual time is clamped so $\tau_k^m\ge\tau_k^a$. In the
manual class, crew switching during the repair stage remains possible, while
DER redispatch, storage dispatch, grid-forming support, black start, and
microgrid islanding can be frozen to represent loss of the control channel.

### 12.2 Conditional expectation model

For class $c\in\{a,m\}$,

$$
\tau_{k,c}^{rep}=\max(0,r_k-\tau_{k,c}^{sw}),
$$

$$
E_{k,c}=S_{k,c}^{sw}\tau_{k,c}^{sw}
+S_{k,c}^{rep}\tau_{k,c}^{rep}.
$$

The cyber-conditioned result is

$$
\boxed{
\mathrm{EENS}=\sum_k\lambda_k
\left[A_kE_{k,a}+(1-A_k)E_{k,m}\right].}
$$

Equivalently,

$$
\mathrm{EENS}=\sum_k\lambda_k
\sum_{c\in\{a,m\}}\Pr(C=c\mid K=k)E_{k,c}.
$$

LOLE, LOLF, nodal EENS, CIF, and CID use the same class-conditioned expectation.
This is mathematically a total-expectation refinement of physical FMEA, not a
new reliability index.

### 12.3 Attribution and bounds

Let $E_k^{dur}$ use manual duration with automatic controls still available.
The implementation reports

$$
\mathrm{EENS}^{perfect}=\sum_k\lambda_kE_{k,a},
$$

$$
\Delta^{cyb,dur}=\sum_k\lambda_k(1-A_k)
(E_k^{dur}-E_{k,a}),
$$

$$
\Delta^{cyb,ctl}=\sum_k\lambda_k(1-A_k)
(E_{k,m}-E_k^{dur}),
$$

$$
\mathrm{EENS}=\mathrm{EENS}^{perfect}
+\Delta^{cyb,dur}+\Delta^{cyb,ctl}.
$$

It also reports the no-automation upper comparison
$\mathrm{EENS}^{noauto}=\sum_k\lambda_kE_{k,m}$ and

$$
\eta_A=\frac{\mathrm{EENS}^{noauto}-\mathrm{EENS}}
{\mathrm{EENS}^{noauto}-\mathrm{EENS}^{perfect}}.
$$

With uniform $A$, $\eta_A=A$ by construction; it becomes decision-useful with
component-specific availability or explicit cyber topology. The cyber-caused
SAIDI share is

$$
\gamma_{SAIDI}=\frac{\mathrm{SAIDI}-\mathrm{SAIDI}^{perfect}}
{\mathrm{SAIDI}}.
$$

The duration/control terms are counterfactual attributions, not guaranteed
nonnegative mathematical measures. Nonnegativity requires the monotonicity
assumption

$$
E_{k,a}\le E_k^{dur}\le E_{k,m}.
$$

If slower switching enables a different topology, or frozen control changes
dispatch in an unexpectedly favorable way, either increment may be negative.
Such a result is diagnostic evidence that the assumed perfect-to-degraded
ordering is not monotone; it must not be silently clamped.

### 12.4 Current boundary

Level 1 has no explicit communication topology, packet latency, cyber-node
power supply, common-cause cyber failures, or learned FDIR policy. Protection
misoperation is handled separately by failure-mode FMEA; its Level-1 cyber
decomposition field is intentionally a zero placeholder.

### 12.5 Implemented three-dimensional factorized screening

**Implemented for deterministic component FMEA.** The GUI/API now separates:

- physical parameters: load scale, switching time, reconfiguration, microgrid,
  storage, grid-forming VSC, and black-start support;
- information parameters: service availability, automatic/manual response
  time, and loss-of-service DER/control freezing;
- intelligent parameters: detection, isolation, restoration-decision,
  restoration-execution, and protection-success screening probabilities.

When information and intelligent screening are enabled, the automatic-class
probability is

$$
A_k^{eff}=A_k^{info}
P_D P_I P_R^{valid}P_R^{exec}P_P^{success}.
$$

Disabled dimensions contribute a factor of one. Component-specific information
availability overrides replace $A_k^{info}$ before multiplication. The existing
automatic/manual consequence pair is then weighted by $A_k^{eff}$ and
$1-A_k^{eff}$.

This is an **independent-factor screening approximation**, not the joint class
generator proposed in Section 18.12. A failed intelligent function is routed to
the existing degraded/manual consequence proxy. In particular,
$P_P^{success}$ does not derive relay pickup, backup-zone expansion, breaker
failure, or DER FRT trajectories. Results therefore report
`joint_class_probability_modelled=false`,
`protection_logic_modelled=false`, and
`protection_frt_reliability_coupled=false`.

## 13. Proposed intelligent cyber-physical reliability model

### 13.1 Why an integrated model is needed

The physical-only question is: given failed equipment, how much load can the
surviving network serve? The integrated question is broader:

1. Was the fault detected, and how quickly?
2. Was the failed section correctly isolated?
3. Did protection clear the intended zone without unnecessary trips?
4. Which restoration actions were observable, communicable, admissible, and
   successfully executed?
5. Could microgrids and DERs form stable islands and sustain them until repair?

Most cyber failures affect reliability indirectly. They lengthen fault-location
and switching time, enlarge the isolated zone, remove feasible restoration
actions, freeze DER setpoints, or suppress grid-forming capability. Therefore,
they belong in stage probabilities, stage durations, and consequence patches.

### 13.2 Three coupled graphs

**Proposed.** Represent the system as:

$$
\mathcal G^{ICP}=(G_p,G_c,G_f,\Pi_{pc},\Pi_{cp}),
$$

where:

- $G_p=(V_p,E_p)$ is the physical AC/DC network;
- $G_c=(V_c,E_c)$ is the information/communication network;
- $G_f$ is a functional dependency graph for detection, isolation,
  restoration, and protection;
- $\Pi_{pc}$ maps cyber services to controlled/observed physical assets;
- $\Pi_{cp}$ maps physical buses to the power supply of cyber assets.

The joint state is

$$
\xi_t=(x_t^p,x_t^c,e_t^c,b_t,\hat x_t,\mathcal A_t),
$$

where $x^p$ and $x^c$ are physical/cyber component states, $e^c$ is cyber-link
quality, $b$ is backup-energy state, $\hat x$ is the intelligent system's belief
about the fault, and $\mathcal A_t$ is the set of currently executable actions.

### 13.3 Cyber function availability

For required function $f$ and cyber state $x^c$, define a Boolean structure
function

$$
\phi_f(x^c)=\mathbf 1[\text{a valid sensor-to-controller-to-actuator service
path exists}].
$$

The function availability is

$$
A_f=\Pr[\phi_f(X^c)=1]
=\sum_{x^c}\phi_f(x^c)\Pr(X^c=x^c).
$$

For a simple series chain, $A_f=\prod_q A_q$. For redundant paths,

$$
A_f=1-\prod_{p\in\mathcal P_f}(1-A_p)
$$

only when the paths are independent. Shared switches, power supplies, control
centers, clocks, and conduits must be represented as common elements or common
cause variables rather than multiplied as independent paths.

The general path expression is

$$
A_f=
\Pr\left(
\bigvee_{p\in\mathcal P_f}
\bigwedge_{q\in p}\{X_q^c=1\}
\right),
$$

evaluated on the joint cyber-state distribution. This remains valid when paths
share elements; the independent-path product does not.

For FLISR, all required functions must hold in the same cyber state:

$$
A_k^{FLISR}=\Pr\left[
\phi_D(X^c)\land\phi_I(X^c)\land\phi_R(X^c)
\right].
$$

This should be evaluated with minimal cut sets, a binary decision diagram, or
Monte Carlo over the explicit graph. Multiplying separately calculated
$A_DA_IA_R$ is generally wrong when services share infrastructure.

### 13.4 Communication quality, not only availability

Binary up/down state is sufficient for slow reliability screening but not for
fast protection and FLISR. A usable command path must satisfy

$$
L_p\le L_f^{max}, \qquad
J_p\le J_f^{max}, \qquad
P_{loss,p}\le P_f^{max},
$$

where $L$, $J$, and $P_{loss}$ are end-to-end latency, jitter, and packet-loss
probability. A path service indicator is

$$
\psi_{f,p}=\phi_{f,p}
\mathbf 1[L_p\le L_f^{max}]
\mathbf 1[J_p\le J_f^{max}]
\mathbf 1[P_{loss,p}\le P_f^{max}].
$$

If $L_p$, $J_p$, or delivery outcome is random, comparing only their means is
not a service-availability model. The service event is

$$
\mathcal Q_{f,p}=
\{\phi_{f,p}=1,\,
L_p\le L_f^{max},\,
J_p\le J_f^{max},\,
P_{loss,p}\le P_f^{max}\},
$$

and QoS-aware function availability is

$$
A_f^{QoS}=\Pr\left(\bigvee_{p\in\mathcal P_f}\mathcal Q_{f,p}\right).
$$

An engineering service-level contract may equivalently require

$$
\Pr(\mathcal Q_{f,p}\mid X^c=x^c)\ge1-\epsilon_f.
$$

The latency/loss variables must be identified as instantaneous outcomes,
distribution parameters, or specified quantiles; these interpretations cannot
be mixed.

Protection traffic requires millisecond-scale bounds; restoration control can
tolerate seconds; supervisory visibility may tolerate longer delays. One global
communication-availability scalar cannot represent these different contracts.

## 14. Intelligent fault detection model

### 14.1 Detection outcomes

**Proposed.** For true fault class $k$ and observation vector $z$, the detector
produces alarm $\hat k$ with confusion matrix

$$
C^D_{k\hat k}=\Pr(\hat k\mid k).
$$

Important outcomes are:

$$
P_D(k)=1-C^D_{k0}, \qquad
P_{miss}(k)=C^D_{k0},
$$

where $0$ denotes no alarm, plus false-alarm probability

$$
P_{FA}=\Pr(\hat k\ne0\mid k=0).
$$

This probability does not enter annual reliability until its exposure basis is
defined. With $\nu_{eval}$ statistically independent decision windows/year,

$$
\lambda_{FA}=\nu_{eval}P_{FA}
$$

is the expected false-alarm frequency. For continuously monitored or serially
correlated detectors, a measured false-alarm rate in alarm/year or a
level-crossing/point-process model is required; treating every sampled time
step as an independent opportunity can overstate nuisance trips by orders of
magnitude.

The detector must be conditioned on cyber observability $o$ and operating
regime $h$:

$$
C^D_{k\hat k}=C^D_{k\hat k}(o,h).
$$

This prevents an offline sensor or topology error from being hidden inside one
average accuracy number.

### 14.2 Detection latency

Use a latency distribution rather than only mean time:

$$
T_k^D\sim F_k^D(t\mid o,h), \qquad
\tau_k^{iso}=T_k^D+T_k^{decision}+T_k^{trip}.
$$

For data-light studies, a two-class model is defensible:

$$
T_k^D=
\begin{cases}
T_{auto}, & \phi_D=1,\\
T_{patrol}(L_k), & \phi_D=0,
\end{cases}
$$

where patrol time depends on the unobserved feeder length. For intelligent
detectors, empirical latency and confusion matrices must come from replayed
fault records, hardware-in-the-loop tests, or validated synthetic faults.

Because physical consequence can change nonlinearly with latency, in general

$$
\mathbb E[E_k(T_k^D)]
\ne E_k(\mathbb E[T_k^D]).
$$

The correct contribution is

$$
\mathrm{EENS}_k
=\lambda_k\int E_k(t)\,dF_k^D(t),
$$

where $E_k(t)$ is the full stage energy consequence conditional on detection
latency $t$. A finite set of mutually exclusive latency classes is a practical
quadrature of this integral.

### 14.3 Reliability consequence

A missed or late detection does not automatically equal full load shed. It
selects a consequence class:

- timely detection: intended protection and FLISR sequence;
- late detection: longer isolation window and possible equipment damage class;
- missed detection: backup protection zone and manual patrol;
- false alarm: nuisance-trip initiating event.

Thus the detector enters reliability through $\pi_c(k)$, $\tau_{k,c,s}$, and
$\Phi_{k,c}$.

## 15. Intelligent fault isolation model

### 15.1 Section identification

**Proposed.** Let $\mathcal Z$ be the set of candidate fault sections or
isolation zones, indexed by $q$, and let $z$ denote the measurement vector.
The isolation engine produces posterior belief

$$
b_q=\Pr(K=q\mid z,G_p,G_c), \qquad
\sum_{q\in\mathcal Z}b_q=1.
$$

The selected isolation zone is

$$
\hat q=\arg\min_{q\in\mathcal Z}
\left[
c_{miss}\Pr(K\notin q\mid z)
+c_{shed}P_{isolated}(q)
\right],
$$

subject to protection selectivity and switch interrupting capability. This
explicitly trades unsafe under-isolation against unnecessarily broad outage.
If candidate $q$ is a set of elementary fault sections, its containment
probability is

$$
\Pr(K\in q\mid z)=\sum_{h\in q}b_h,\qquad
\Pr(K\notin q\mid z)=1-\sum_{h\in q}b_h.
$$

### 15.2 Isolation success and zone expansion

Let $p_I(k,c)$ be the probability that the intended boundaries open and the
fault is contained under cyber class $c$. Then

$$
\Pr(Z_k=Z_k^{primary})=p_I(k,c),
$$

$$
\Pr(Z_k=Z_k^{backup})=1-p_I(k,c).
$$

The failed-isolation consequence uses the protection-zone expansion mutation.
For a sequence of demanded devices $q\in Q_k$,

$$
p_I(k,c)=\Pr\left[\bigwedge_{q\in Q_k}
(\text{command path available})_q
\land(\text{device operates})_q\right].
$$

Shared communication paths and common DC supplies must be evaluated jointly.

## 16. Intelligent restoration model

### 16.1 Decision problem

**Proposed.** Restoration selects switching, DER dispatch, islanding, and load
pickup actions under partial observability:

$$
\pi_R:\ (\hat x_t,G_p,G_c,E_t^{st})\mapsto a_t.
$$

A one-step risk-aware formulation is

$$
\min_{a,y}\quad
\sum_i w_i p_i^{sh}
+c_{sw}\sum_\ell |z_\ell-z_\ell^0|
+c_{risk}\,\mathcal R(a,\hat x),
$$

At minimum, the abstract risk term must identify a safety violation event:

$$
\mathcal R(a,\hat x)
=\Pr(g(a,X)>0\mid\hat x),
$$

where $g$ may encode incomplete fault isolation, overload, unsafe energization,
protection miscoordination, voltage violation, or synchronization failure.
Different hazards may be combined as a weighted vector, but the weights and
probability model must be explicit.

The optimization is subject to the physical restoration model, protection interlocks, and
communication/action availability:

$$
z_\ell^{close}\le\phi_{cmd(\ell)}(x^c),
$$

$$
p_r^{dispatchable}\le \overline P_r\phi_{ctrl(r)}(x^c),
$$

$$
y_{island}\le\phi_{GFM}(x^c)x_{GFM}^p.
$$

An unavailable command path does not necessarily make an action impossible: it
may convert remote operation into a crew action with a travel/operation delay.
Therefore each action has both feasibility and timing:

$$
T_a=
\begin{cases}
T_a^{remote}, & \phi_{cmd(a)}=1,\\
T_a^{crew}+T_a^{manual}, & \phi_{cmd(a)}=0\text{ and manually operable},\\
\infty, & \text{otherwise}.
\end{cases}
$$

### 16.2 Policy success and unsafe actions

The reliability study must distinguish optimizer feasibility from intelligent
policy performance. Let

$$
p_R^{valid}=\Pr(a\in\mathcal A_{safe}\mid\hat x,x),
$$

$$
p_R^{exec}=\Pr(\text{all required actions execute}\mid a,x^c).
$$

The automated restoration class probability is

$$
\pi_{auto}(k)=
\Pr(D_k\cap I_k\cap R_k^{valid}\cap R_k^{exec}\mid K=k).
$$

Only under conditional independence may it be factorized as

$$
\pi_{auto}(k)=
\Pr(D_k\mid k)\,
\Pr(I_k\mid k)\,
\Pr(R_k^{valid}\mid k)\,
\Pr(R_k^{exec}\mid k).
$$

The product
$A_k^{FLISR}P_D(k)p_I(k)p_R^{valid}p_R^{exec}$ is therefore only a screening
approximation and can double-count shared sensor, communication, control, or
power-supply dependencies. Invalid or unverified actions must route to a
conservative manual/blocked class, never be credited as successful restoration.

### 16.3 Sequential or learning-based policy

For multi-step restoration, use a constrained POMDP or model-predictive policy:

$$
\min_\pi\mathbb E_\pi\left[
\sum_t\gamma^t\left(\sum_i w_i p_{i,t}^{sh}+c(a_t)\right)
\right]
$$

subject to hard electrical and protection constraints for every executed
action. A learned policy may propose candidate actions, but a deterministic
safety filter must validate radiality, voltage, thermal limits, synchronization,
interlocks, and DER capability before execution. Reliability credit belongs to
the validated composite policy, not the unconstrained model.

## 17. Intelligent protection model

### 17.1 Dependability and security

**Proposed, partly supported by current failure-mode FMEA.** Protection has two
distinct failure families:

- dependability failure: fail-to-trip on a real demand;
- security failure: nuisance trip with no in-zone fault.

For protection zone $z$,

$$
\nu_z=\sum_{k\in\mathcal K_z}\lambda_k,
\qquad
\lambda_z^{FT}=\nu_z p_z^{FT}.
$$

This sum is valid only when every $k\in\mathcal K_z$ produces exactly one demand
on the modeled protection function and protection zones are not double-counted.
Overlapping zones, breaker-failure schemes, and multiple relay elements require
an event tree identifying which functions are demanded by each physical fault.

The expected consequence of fail-to-trip is

$$
\mathrm{EENS}^{FT}=\sum_z\lambda_z^{FT}
\sum_s S_{z,s}^{backup}\tau_{z,s}^{backup}.
$$

Nuisance trips are passive initiating events:

$$
\mathrm{EENS}^{NT}=\sum_z\lambda_z^{NT}
\sum_s S_{z,s}^{trip}\tau_{z,s}^{trip}.
$$

Protection dependability may require both local IED health and a communication
contract:

$$
p_z^{FT}=1-\Pr(\text{relay healthy}\land
\text{measurements valid}\land\text{trip path timely}).
$$

A fuller demanded-success event is

$$
\mathcal S_z=
R_z\cap M_z\cap C_z\cap B_z\cap S_z\cap A_z,
$$

where $R_z$ is relay logic health, $M_z$ measurement validity, $C_z$ timely
communication/trip path when required, $B_z$ breaker mechanical operation,
$S_z$ correct/selective settings, and $A_z$ auxiliary-power availability. Thus

$$
p_z^{FT}=\Pr(\mathcal S_z^c\mid\text{protection demand}).
$$

Local primary protection should not be made dependent on wide-area
communication unless the actual scheme requires it. Otherwise the model will
systematically overstate cyber-protection coupling.

### 17.2 Adaptive protection with DERs

Microgrid mode, inverter fault-current limits, and topology changes alter relay
reach and coordination. Let $\sigma$ denote grid-connected/islanded topology and
$\theta_z$ the active setting group. Mis-coordination probability is

$$
p_z^{mis}=\Pr(\theta_z\ne\theta_z^*(\sigma)
\ \lor\ T_{update}>T_{required}).
$$

This creates three modes: setting-update communication loss, incorrect setting
selection, and insufficient fault current for pickup. Their consequences should
be evaluated with short-circuit/protection models to determine the backup zone,
then passed to reliability as a zone mutation and duration class.

Section 17 is the reliability-event abstraction. Section 18 refines its event
probabilities, clearing times, topology mutations, and DER consequences from
relay and FRT event logic; it does not change the implementation status of the
current reliability evaluators.

## 18. Protection-device logic and DER fault-ride-through reliability interface

### 18.1 Implementation status and exact boundary

Protection and DER ride-through are not wholly absent from the repository, but
they are not yet integrated into the reliability consequence path.

| Capability | Current status | Reliability implication |
|---|---|---|
| Device-level voltage/frequency violation timers, trip, reconnect qualification, and reconnect power ramp | **Implemented, opt-in, in dynamics** | Dynamic trajectories can contain DER trip/reconnect events, but reliability FMEA/MC does not invoke this state machine |
| GFL/GFM dynamic models, current limits, limiter priorities, volt-var, and frequency-watt controls | **Implemented, opt-in, in dynamics** | Available for transient studies; not converted into contingency-class probabilities or EENS stages |
| Detailed short-circuit converter contribution | **Implemented in short-circuit analysis** | GFL is represented as a current source and GFM as a voltage source behind impedance; this is a static fault snapshot, not relay/FRT event evolution |
| Switch capabilities, fuse/recloser/sectionalizer metadata, reclose intervals, and lockout data | **Represented in the rich model** | Data exist, but reliability does not simulate time-current curves, reclose shots, fuse saving, or sectionalizer counts |
| Fail-to-trip, nuisance-trip, fail-to-open/close, protection-zone expansion, and interlock checks | **Implemented as reliability event abstractions** | Consequences are available, but pickup, timing, selectivity, and DER-dependent fault current are not derived |
| Full relay logic, primary/backup coordination, breaker-failure sequence, DER FRT/protection feedback, and FRT-conditioned EENS | **Proposed** | Required before reliability results can claim protection-logic or DER-FRT coverage |

The current dynamics IEEE-1547 block uses filtered positive-sequence voltage
magnitude and frequency inferred from voltage angle, continuous band-specific
violation timers, approximate category default tables, trip/reconnect states,
and a reconnect qualification timer. It does **not** by itself provide:

- a full mandatory-ride-through / permissive-trip / momentary-cessation state
  taxonomy;
- feeder relay pickup and time-current logic;
- directional, distance, differential, breaker-failure, or recloser
  coordination;
- reliability weighting of the resulting dynamic event sequence.

Accordingly, this section is a **proposed reliability interface around existing
dynamic and short-circuit kernels**, not a claim that the integrated model is
currently shipped.

Unless a reliability result reports
`protection_frt_reliability_coupled=true`, the dynamics IEEE-1547 and
short-circuit capabilities are separate analyses, not contributors to that
result's EENS, LOLE, LOLF, SAIDI, or restoration-stage DER availability. The
flag is valid only when the reported metric actually consumes the coupled event
classes or trajectories; merely running a separate dynamic study is
insufficient.

### 18.2 Hybrid protection-FRT state

Let the event-time state be

$$
\zeta(t)=
\left(
x^{dyn}(t),\,
\sigma(t),\,
\chi(t),\,
\kappa(t),\,
\Omega(t),\,
s^{br}(t),\,
s^{rec}(t),\,
\vartheta(t)
\right),
$$

where:

| State | Meaning |
|---|---|
| $x^{dyn}(t)$ | electrical/controller differential state |
| $\sigma(t)$ | topology and grid-connected/islanded mode |
| $\chi_r(t)\in\{0,1\}$ | DER $r$ electrically connected state |
| $\kappa_r(t)$ | composite DER mode $(\kappa_r^{ctrl},\kappa_r^{FRT},\kappa_r^{lim})$: control family, FRT/connection behavior, and limiter state |
| $\Omega_z(t)$ | protection operating quantity or timer for element $z$ |
| $s_z^{br}(t)$ | relay command, breaker contact, and arc-clearing state |
| $s_z^{rec}(t)$ | recloser shot count, dead-time state, and lockout state |
| $\vartheta_z(t)$ | active protection setting group |

Use $\delta_i(t)$ for electrical voltage angle and reserve $\vartheta_z$ for
protection settings; using one $\theta$ for both creates an avoidable ambiguity.
The composite DER mode avoids treating GFL/GFM, ride-through/trip, and
current-limited/unlimited as mutually exclusive labels. For example, a DER may
simultaneously be GFM-controlled, in mandatory ride-through, and current
limited.
Between discrete events, the state follows

$$
\dot x^{dyn}=F_{\sigma,\kappa}(x^{dyn},y,u,w),
$$

$$
0=G_{\sigma,\kappa}(x^{dyn},y,u,w),
$$

where $y$ contains algebraic network variables such as bus voltages, branch
currents, and interface powers; $u$ is the commanded/control input and $w$ is
the exogenous disturbance. This notation avoids using $v$ for both the complete
algebraic state and voltage alone.

Discrete guards generate trips, mode transitions, breaker openings, reclose
attempts, and reconnections:

$$
\zeta(t^+)=\mathcal R_e(\zeta(t^-))
\quad\text{when}\quad
g_e(\zeta(t^-))=0.
$$

This hybrid-automaton semantics is necessary because a protection action changes
topology discontinuously, while DER controllers and relay timers evolve
continuously between actions.

### 18.3 DER voltage/frequency ride-through automaton

For DER $r$ at bus $i(r)$, let

$$
V_r(t)=|V_{i(r)}^{+}(t)|,\qquad
f_r(t)=f_{i(r)}^{meas}(t),
$$

where $V^+$ is positive-sequence voltage and $f^{meas}$ is the specified
filtered frequency measurement. Positive-sequence use matches the present
dynamics implementation; unbalanced FRT requires phase/sequence extensions.

Let $\mathcal J_{r,V}$ and $\mathcal J_{r,f}$ be configured voltage and frequency
bands. For band $j$, define its active predicate $h_{r,j}(t)\in\{0,1\}$ and
continuous violation timer

$$
D_{r,j}(t)=
\begin{cases}
t-\displaystyle\sup\left(
\{s\in[t_0,t]:h_{r,j}(s)=0\}\cup\{t_0\}
\right), & h_{r,j}(t)=1,\\
0, & h_{r,j}(t)=0.
\end{cases}
$$

This definition resets a band timer when its condition clears. It is stricter
than accumulating all abnormal-voltage time into one scalar, which would
incorrectly combine separate excursions and separate bands.

Let each band be classified as continuous operation, mandatory ride-through,
permissive action, momentary cessation, or mandatory trip according to the
applicable interconnection profile and actual device settings. Let
$\mathcal J_r^{trip}\subseteq\mathcal J_{r,V}\cup\mathcal J_{r,f}$ contain only
bands for which the configured device logic issues a trip. A configured trip
guard is

$$
\mathcal G_r^{trip}(t)=
\bigvee_{j\in\mathcal J_r^{trip}}
\left[D_{r,j}(t)\ge T_{r,j}^{trip}\right].
$$

Overlapping band predicates require an explicit priority rule in $\Psi_r$;
otherwise two simultaneously active bands can prescribe incompatible actions.
Permissive-action bands do not become deterministic trip bands unless the
actual utility/device setting selects that behavior.

The category alone does not determine $T_{r,j}^{trip}$. The configured behavior
is a parameterized mapping

$$
T_{r,j}^{trip}=\mathcal T_{r,j}\left(
\text{standard version/category},\,
\text{interconnection profile},\,
\text{utility setting},\,
\text{manufacturer/firmware evidence}
\right).
$$

The selected point must lie within the applicable requirements, but the
standard name by itself is not an actual device trip curve.

The generic mode transition is

$$
\kappa_r(t^+)=
\Psi_r\left(
\kappa_r(t^-),V_r(t),f_r(t),
\{D_{r,j}(t)\}_j,\eta_r^{device}
\right),
$$

where $\eta_r^{device}$ includes category, utility settings, manufacturer
behavior, firmware, protection enable state, and uncertainty. Connection state
obeys

$$
\chi_r(t^+)=
\begin{cases}
0, & \mathcal G_r^{trip}(t)=1,\\
1, & \mathcal G_r^{reconnect}(t)=1,\\
\chi_r(t^-), & \text{otherwise}.
\end{cases}
$$

The simple pair of exposure integrals
$D_{r,V}=\int\mathbf 1[V\in\mathcal V_{abn}]dt$ and
$D_{r,f}=\int\mathbf 1[f\in\mathcal F_{abn}]dt$ is acceptable only when the
device has one abnormal band per quantity. Piecewise trip envelopes require the
per-band timers above.

### 18.4 Ride-through, momentary cessation, and trip are different states

DER output must be conditioned on its dynamic mode:

$$
(p_r(t),q_r(t))=
\begin{cases}
(p_r^{cmd},q_r^{cmd}), & \kappa_r^{FRT}=\mathrm{normal},\\
(p_r^{FRT},q_r^{FRT}), & \kappa_r^{FRT}=\mathrm{ride\ through},\\
(p_r^{MC},q_r^{MC}), & \kappa_r^{FRT}=\mathrm{momentary\ cessation},\\
(0,0), & \kappa_r^{FRT}\in\{\mathrm{tripped},\mathrm{blocked},
\mathrm{reconnect\ wait}\},\\
a_r(t)(p_r^{avail},q_r^{avail}), & \kappa_r^{FRT}=\mathrm{ramping}.
\end{cases}
$$

The cessation policy $(p_r^{MC},q_r^{MC})$ is device-specific. It may be
$(0,0)$, active-power cessation with reactive support, or another certified
fallback. The reliability model must not assume that all connected DERs support
voltage, nor that momentary cessation is equivalent to physical disconnection.

The current dynamics state machine explicitly represents connected/tripped,
reconnect qualification, and a restore scale. A separate momentary-cessation
mode is not currently exposed by the reliability module and must be modeled or
declared absent.

The transition map $\Psi_r$ must define both entry and exit guards for momentary
cessation, including any recovery dwell, hysteresis, and delayed escalation to
trip. Without those guards, the sequence
$\mathrm{ride\ through}\leftrightarrow\mathrm{momentary\ cessation}$ can chatter
at a band boundary and the terminal FRT class is not unique.

### 18.5 Fault-period inverter current and GFL/GFM behavior

Let $i_{d,r}^{ref}$ and $i_{q,r}^{ref}$ be the controller current commands. The
common current limit is

$$
i_{d,r}^2+i_{q,r}^2\le(I_r^{max})^2.
$$

With active-current priority,

$$
i_{d,r}=\operatorname{sat}(i_{d,r}^{ref},I_r^{max}),
$$

$$
i_{q,r}=
\operatorname{sat}\left(
i_{q,r}^{ref},
\sqrt{(I_r^{max})^2-i_{d,r}^2}
\right).
$$

With reactive-current priority,

$$
i_{q,r}=\operatorname{sat}(i_{q,r}^{ref},I_r^{max}),
$$

$$
i_{d,r}=
\operatorname{sat}\left(
i_{d,r}^{ref},
\sqrt{(I_r^{max})^2-i_{q,r}^2}
\right).
$$

The saturation operator preserves the sign of its reference and clips its
magnitude at the stated nonnegative bound:

$$
\operatorname{sat}(x,\bar x)
=\operatorname{sgn}(x)\min(|x|,\bar x),\qquad \bar x\ge0.
$$

A voltage-support command may be represented by

$$
i_{q,r}^{ref}
=\operatorname{sat}\left(
K_{q,r}(V_r^{ref}-V_r),\overline I_{q,r}
\right),
$$

with deadband, filtering, delay, and ramp limits included when supported.

For a balanced three-phase RMS model,

$$
p_r^2+q_r^2
\le\left(\sqrt3\,V_{LL,r}I_r^{max}\right)^2.
$$

In per unit this becomes $p_r^2+q_r^2\le V_r^2(I_r^{max})^2$ when voltage,
current, and power share consistent three-phase bases. In the dimensional form,
$V_{LL,r}$ is line-to-line RMS voltage, $I_r^{max}$ is line-current RMS, and
$p_r,q_r$ are total three-phase powers. Mixing phase voltage, line voltage,
single-phase power, and three-phase power creates factors of $\sqrt3$ or three.
This fault-period limit is voltage-dependent and is not equivalent to the
steady nameplate circle.

GFL and GFM require different network representations:

- **GFL:** a PLL-synchronized, controlled current source. Low voltage, PLL loss,
  current-command priority, and cessation/trip determine its contribution.
- **GFM:** a controlled voltage source behind filter/virtual impedance until
  its limiter changes the effective mode. It is not an unlimited voltage source
  during a fault.

A minimal GFM mode split is

$$
\kappa_r(t)=
\begin{cases}
(\mathrm{GFM},\kappa_r^{FRT},\mathrm{voltage\ forming}),
& |i_r^{ref}(t)|\le I_r^{max},\\
(\mathrm{GFM},\kappa_r^{FRT},\mathrm{current\ limited}),
& |i_r^{ref}(t)|>I_r^{max}.
\end{cases}
$$

Protection reach and island-support claims must use the post-limit mode; the
label `GFM` alone does not certify unlimited voltage support or stable islanding.

The repository already contains GFL/GFM dynamic current limiters and the
detailed short-circuit model distinguishes current-source GFL from
voltage-source GFM. The reliability model still lacks a rule that converts
their trajectory into protection outcome and customer consequence.

For unbalanced faults, one scalar positive-sequence current is insufficient.
Let $A_{012\rightarrow abc}$ be the symmetrical-component transformation. A
direct hardware-current constraint is

$$
\begin{bmatrix}I_{a,r}\\I_{b,r}\\I_{c,r}\end{bmatrix}
=A_{012\rightarrow abc}
\begin{bmatrix}I_{r}^{0}\\I_{r}^{+}\\I_{r}^{-}\end{bmatrix},
\qquad
\max_{\phi\in\{a,b,c\}}|I_{\phi,r}|\le I_r^{max},
$$

with all phasors on the same RMS base. Controller-specific negative- and
zero-sequence objectives determine how the feasible current is allocated; an
arbitrary weighted sum of sequence magnitudes is not a generally valid
converter current limit. Results using only positive sequence must declare
that limitation.

### 18.6 Protection measurement and pickup logic

The set of predicates $\{g_z\}$ is the set of relay elements actually installed,
enabled, and assigned to the active setting group. Overcurrent, distance,
differential, voltage, frequency, and wide-area elements are alternatives or
complements determined by the studied scheme, not a universal bundle applied to
every device.

For protection function $z$, define the measured/filtered signal

$$
y_z(t)=
\left(
\widetilde I_z,\widetilde V_z,\widetilde P_z,\widetilde Q_z,
\widetilde f_z,\widetilde{\dot f}_z,
\widetilde I_z^{diff},Z_z^{app}
\right).
$$

Measurement transformers, filtering, sampling, phasor estimation, time
synchronization, and communication delay are part of the observation map:

$$
y_z(t)=\mathcal H_z\left(x^{dyn}_{[0,t]},x^c_{[0,t]},
\epsilon_z^{CT/PT},\tau_z^{comm}\right).
$$

When CT saturation is enabled, $\mathcal H_z$ is a dynamic nonlinear map, for
example

$$
\widetilde I_z(t)=
\mathcal H_z^{CT}\left(I_{z,[0,t]},\Phi_z^{core},R_z^{burden},\ldots\right),
$$

not an additive static error. If this map is absent, the result must state that
CT/PT effects are only static, bounded, or omitted and that CT saturation
dynamics are not modeled.

The generic pickup predicate is

$$
\rho_z(t)=
\mathbf 1[g_z(y_z(t),\vartheta_z,\sigma(t))\ge0].
$$

Representative functions include:

**Overcurrent**

$$
\rho_z^{OC}(t)=\mathbf 1[|\widetilde I_z(t)|\ge I_z^{pickup}].
$$

**Directional overcurrent**

$$
\rho_z^{DOC}(t)=
\rho_z^{OC}(t)
\mathbf 1[\mathcal V_z^{pol}(t)=1]
\mathbf 1\left[
\operatorname{Re}\left(
\widetilde V_z^{pol}\widetilde I_z^*e^{-j\phi_z}
\right)>0
\right].
$$

Here $\mathcal V_z^{pol}$ is the polarizing-signal validity guard. It may encode
live-voltage validity, memory-voltage availability, blocking, or an installed
fallback. This polarizing-torque form is preferable to a bare angle interval
when voltage collapses or memory polarization is used.

**Distance**

$$
Z_z^{app}(t)=\frac{\widetilde V_z(t)}{\widetilde I_z(t)},\qquad
\rho_{z,h}^{dist}(t)=\mathbf 1[Z_z^{app}(t)\in\mathcal Z_{z,h}],
$$

where $\mathcal Z_{z,h}$ is the configured mho/quadrilateral zone. The apparent
impedance is evaluated only when the current/polarization validity guard is
satisfied; it is not defined by dividing through zero or invalid phasors.

**Differential**

$$
\rho_z^{diff}(t)=
\mathbf 1\left[
|I_z^{end1}+I_z^{end2}|
\ge I_z^{min}+k_z\frac{|I_z^{end1}|+|I_z^{end2}|}{2}
\right].
$$

Here $I_z^{end1}$ and $I_z^{end2}$ use aligned polarity references and a common
current base; otherwise the operate/restraint expression is not meaningful.
This is a single channel/phasor expression. A three-phase scheme must state
whether the logic is applied per phase, to sequence components, or to another
installed operate/restraint combination.

Voltage, frequency, ROCOF, anti-islanding, and synchronization functions are
defined by their corresponding measured-signal guards. A reliability study
may use only the functions actually installed; adding a generic wide-area
dependency to local primary protection would overstate cyber risk.

### 18.7 Definite-time and inverse-time operating quantities

For definite-time protection with continuous-pickup semantics,

$$
\dot D_z(t)=
\begin{cases}
1, & \rho_z(t)=1,\\
-D_z/T_z^{reset}, & \rho_z(t)=0,
\end{cases}
$$

with instantaneous reset represented by $D_z(t^+)=0$ when pickup clears. The
relay issues a trip when

$$
D_z(t)\ge T_z^{delay}.
$$

For inverse-time overcurrent, a generic curve is

$$
T_z^{curve}(M)=
\mathrm{TMS}_z
\left[
\frac{A_z}{M^{p_z}-1}+B_z
\right]
+T_z^{add},
\qquad
M=\frac{|\widetilde I_z|}{I_z^{pickup}}>1.
$$

When current changes during the fault, integrate the operating quantity:

$$
\dot\Omega_z(t)=
\begin{cases}
1/T_z^{curve}(M(t)), & M(t)>1,\\
-\Omega_z/T_z^{reset}, & M(t)\le1,
\end{cases}
$$

and issue the relay command at

$$
T_z^{relay}=\inf\{t:\Omega_z(t)\ge1\}.
$$

This integral is essential when DER current limiting, DER trip, voltage support,
or topology change modifies current before the relay operates.

The electrical fault is not cleared at relay-command time. The clearing chain
is

$$
T_z^{clear}=
T_z^{relay}
+T_z^{channel}
+T_z^{tripcoil}
+T_z^{mechanical}
+T_z^{arc}.
$$

Each term may be random and cyber-dependent only when the installed scheme
actually uses a communication channel. For a wholly local hardwired trip,
$T_z^{channel}=0$ (or the term is omitted); for pilot or transfer-trip schemes,
$T_z^{channel}$ is conditioned on the relevant communication-service state and
delivery contract.

All $T_z^{curve}$, timer, channel, breaker, arc, coordination, dead-time, and
reconnection quantities in this event interface must use one declared time
base, normally seconds. $M$ and $\Omega_z$ are dimensionless; $D_z$ and every
$T$ quantity have units of time. Curve coefficients must follow the selected
IEC/IEEE/manufacturer family rather than mix constants from different curve
definitions.

### 18.8 Primary/backup coordination and breaker failure

For fault $k$, let $\mathcal P_k^{pri}$ and $\mathcal P_k^{bak}$ be installed
primary and backup clearing functions. Define

$$
T_k^{pri}=\min_{z\in\mathcal P_k^{pri}}T_z^{clear}(k),\qquad
T_k^{bak}=\min_{z\in\mathcal P_k^{bak}}T_z^{clear}(k).
$$

Set $T_z^{clear}=+\infty$ for an element that never issues an effective command
or whose breaker never interrupts the fault within the study horizon.

Separate the physical clearing outcome from the coordination diagnostic. Let

$$
\mathcal E_k^{pri}
=\{\text{the fault is de-energized by a designated primary device by }
T_k^{clear,max}\},
$$

and define satisfactory coordination as

$$
\mathcal S_k^{coord}=\mathcal E_k^{pri}\cap
\{T_k^{bak}-T_k^{pri}\ge\Delta T_k^{coord}\}.
$$

The distinction matters: a primary device can clear the fault while the
calculated backup margin is deficient. That is an actual primary-cleared
outcome with a coordination-warning attribute, not an unclassified sample.

Define the backup-cleared event

$$
\mathcal E_k^{bak}
=\{\text{the fault is de-energized by a designated backup device by }
T_k^{backup,max}\}.
$$

The following are useful raw diagnostic events:

$$
\mathcal F_k^{mis}=
\{\text{a healthy or non-designated zone trips before final fault clearing}\},
$$

$$
\mathcal S_k^{sel}=\mathcal E_k^{pri}\cap(\mathcal F_k^{mis})^c,
$$

$$
\mathcal F_k^{FT}=
\{\text{the fault remains energized at }T_k^{backup,max}\}.
$$

$$
\mathcal E_k^{self}=
\{\text{the fault extinguishes before any protection operation}\}.
$$

After a trip command, breaker failure is detected when current persists:

$$
\mathcal F_{z}^{BF}=
\left\{
u_z^{trip}(t_z^{cmd})=1,\,
s_z^{br}\ne\mathrm{open},\,
\inf_{\tau\in[t_z^{cmd},t_z^{cmd}+T_z^{BF}]}
|\widetilde I_z(\tau)|>I_z^{open}
\right\}.
$$

This event starts the breaker-failure backup scheme and expands the cleared
zone. Mechanical fail-to-open probability remains a separate demanded failure;
it should not be multiplied independently if already embedded in a calibrated
breaker-failure event probability.

These raw events can overlap: for example, breaker failure can cause backup
clearing, and a backup device can also disconnect a healthy zone. Reliability
classes must therefore be assigned by an explicit event tree. For a real fault,
one defensible priority is:

1. `fail to clear` if the fault remains energized at
   $T_k^{backup,max}$;
2. otherwise `miscoordinated` if any non-required healthy zone was tripped;
3. otherwise `backup cleared` if $\mathcal E_k^{bak}$ occurred;
4. otherwise `primary cleared` if $\mathcal E_k^{pri}$ occurred;
5. otherwise `transient self-cleared` if the fault extinguished before any
   protection operation.

Breaker failure, coordination-margin violation, and DER-induced underreach are
recorded as cause attributes on the selected terminal class, rather than added
as overlapping terminal probabilities. Nuisance trip is a separate initiating
event with $K=0$. Under this ordering, the terminal values of

$$
c_P\in\{
\mathrm{primary\ cleared},
\mathrm{backup\ cleared},
\mathrm{miscoordinated},
\mathrm{fail\ to\ clear},
\mathrm{transient\ self\ cleared},
\mathrm{nuisance\ trip}
\}
$$

are mutually exclusive. They map directly to different topology mutations,
equipment-damage states, and repair durations. A study may use a different
priority only if it publishes an equally exhaustive, mutually exclusive event
tree.

Formally, the terminal label and nonexclusive cause attributes are separate:

$$
C_P=\mathcal E_{tree}\left(
\mathcal F_k^{FT},\mathcal F_k^{mis},
\mathcal E_k^{bak},\mathcal E_k^{pri},\mathcal E_k^{self}
\right),
$$

$$
A_P\subseteq\{
\mathrm{breaker\ failure},\mathrm{underreach},\mathrm{overreach},
\mathrm{margin\ violation},\mathrm{setting\ error},\ldots
\}.
$$

Reliability probabilities are normalized over $C_P$; $A_P$ supports attribution
and diagnostics and must not be added as another set of terminal probabilities.

### 18.9 Recloser, fuse, sectionalizer, and anti-islanding sequence

Let recloser $z$ have shot count $n_z$, dead times
$\{\Delta t_{z,h}^{dead}\}$, maximum shots $N_z^{max}$, and lockout state
$L_z\in\{0,1\}$. After shot $h$,

$$
t_{z,h}^{reclose}
=t_{z,h}^{open}+\Delta t_{z,h}^{dead},
$$

provided

$$
h<N_z^{max},\qquad L_z=0,\qquad
\mathcal S_z^{sync}=1,\qquad
\mathcal S_z^{interlock}=1.
$$

A reclose success probability cannot be a universal device constant. It should
be conditioned on fault type and clearing:

$$
p_{z,h}^{rec}(k)=
\Pr(\text{fault extinguished before shot }h
\mid k,T_{z,h}^{dead},\text{weather}).
$$

Permanent faults lead to repeated trip or lockout; transient faults may clear
during dead time. Fuse-saving/blowing and sectionalizer behavior require event
ordering:

$$
T_{recloser}^{fast}(I)
<T_{fuse}^{min\text{-}melt}(I)
$$

for a fuse-saving fast shot. A fuse-blowing delayed sequence requires the
separate total-clear curve, for example

$$
T_{fuse}^{total\text{-}clear}(I)
<T_{recloser}^{delayed}(I).
$$

Minimum-melt and total-clear curves are not interchangeable. A sectionalizer
opens only after its counted upstream interruptions reach the configured
threshold during dead time.

DERs can maintain voltage/current in an intended dead interval, defeat
de-energized-operation assumptions, or form an unintended island. Reclose must
therefore also check

$$
\mathcal S_z^{dead}=
\mathbf 1[|V_{line}(t)|\le V_z^{dead,max}],
$$

or synchronization conditions

$$
|\Delta V|\le\Delta V^{max},\quad
|\Delta f|\le\Delta f^{max},\quad
|\Delta\delta|\le\Delta\delta^{max}.
$$

Anti-islanding and transfer-trip behavior determines whether DER remains on the
isolated section. Reliability stages must use the resulting shot/lockout and
DER states rather than assume immediate successful reclose.

### 18.10 DER-protection closed loop

For fault $k$, measured relay current is a network function of grid and DER
contributions:

$$
I_z(t;k)=
I_z^{grid}(t;k,\sigma(t))
+\sum_{r\in\mathcal R}
\chi_r(t)H_{zr}(\sigma(t),k)I_r^{fault}(t).
$$

Here $H_{zr}$ is a topology- and fault-dependent transfer/sensitivity operator,
not generally a fixed scalar across switching events. This additive expression
is a linearized/superposition interface; when converter controls and network
algebraic equations are nonlinear, $I_z$ must instead come from the coupled
network solve. Protection time depends on the DER trajectory,

$$
T_z(k)=
\mathcal T_z\left(
\{I_z(t;k),V_z(t;k)\}_{0\le t\le T_z},
\vartheta_z
\right),
$$

while DER survival depends on clearing:

$$
\Pr[\chi_r(T_k^{clear})=1\mid k]
=
\Pr\left[
\mathcal G_r^{trip}(t)=0
\quad\forall t<T_k^{clear}
\mid k
\right].
$$

Thus

$$
T_k^{clear}
=\mathcal T_k\left(
\{\chi_r(t),\kappa_r(t),I_r^{max}\}_{r\in\mathcal R}
\right)
$$

is an implicit hybrid event problem. A causal event-driven solution is:

1. initialize fault, topology, DER modes, relay timers, and breaker states;
2. solve the short-circuit/dynamic network until the next candidate guard;
3. advance relay and FRT timers on accepted integration steps;
4. execute the earliest DER mode change, relay trip, breaker opening, fault
   extinction, reclose, or lockout event;
5. rebuild topology/network equations and repeat until a stable post-fault
   class is reached or a fail-to-clear horizon is exceeded.

This is preferable to a non-causal fixed-point iteration over final clearing
time. The existing dynamic event engine is the natural substrate; the static
short-circuit solver is suitable for screening pickup and initial current but
cannot alone resolve the full loop.

#### Cyber dependencies of protection-FRT

Cyber coupling is function-specific. Representative contracts are

$$
T_z^{channel}=T_z^{channel}(x^c,e^c),
$$

$$
\vartheta_z(t^+)=
\begin{cases}
\vartheta_z^*(\sigma(t)), & \phi_{setting,z}(x^c)=1,\\
\vartheta_z(t^-), & \phi_{setting,z}(x^c)=0,
\end{cases}
$$

$$
u_z^{pilot}(t)\le\phi_{pilot,z}(x^c),\qquad
u_r^{remote\ trip}(t)\le\phi_{DERtrip,r}(x^c).
$$

Here $e^c$ contains latency/loss quality and $\phi$ is the relevant service
structure function. Local autonomous relay, FRT, and inverter-limiter functions
remain available when their local measurement, auxiliary power, and device
logic are healthy; they must not be made dependent on SCADA or wide-area
communication merely because those networks exist.

### 18.11 DER reconnection and post-fault availability

Let $\mathcal V_r^{rec}$ and $\mathcal F_r^{rec}$ be the configured reconnect
windows. Continuous qualification is

$$
\mathcal C_r^{rec}(t)=
\left\{
V_r(\tau)\in\mathcal V_r^{rec},\,
f_r(\tau)\in\mathcal F_r^{rec}
\quad
\forall\tau\in[t-T_r^{stable},t]
\right\}.
$$

Define the first completed qualification time as

$$
T_r^{qualify}=\inf\{t:\mathcal C_r^{rec}(t)=1\}.
$$

Set $T_r^{qualify}=+\infty$ when the qualification event never occurs within
the study horizon.

If synchronization is required,

$$
T_r^{sync}
=\inf\left\{
t:
|\Delta V_r(t)|\le\Delta V_r^{max},\,
|\Delta f_r(t)|\le\Delta f_r^{max},\,
|\Delta\delta_r(t)|\le\Delta\delta_r^{max}
\right\}.
$$

Online time is

$$
T_r^{online}
=\max(T_r^{qualify},T_r^{sync})
+T_r^{close},
$$

with $T_r^{sync}=0$ when synchronization is not applicable. Let $a_r(t)$ be the
availability scale. For a DER that did not disconnect, $a_r(t)=1$. For a DER
that reconnects with $T_r^{ramp}>0$,

$$
a_r(t)=
\operatorname{clip}\left(
\frac{t-T_r^{online}}{T_r^{ramp}},0,1
\right).
$$

For $T_r^{ramp}=0$, define $a_r(t)=\mathbf 1[t\ge T_r^{online}]$ rather than
evaluate the quotient.

Then reliability-stage available power is

$$
\overline p_{r,t}^{avail}
=
\chi_r(t)a_r(t)
\min\left(
\overline p_{r,t}^{weather},
\overline p_r^{control}
\right).
$$

The present dynamics model already qualifies reconnect inside a continuous
voltage/frequency window and exposes a restore scale, although device types may
apply ramp behavior differently. Reliability currently restores DER capability
without consuming this dynamic trajectory, which can overstate short
restoration-stage support.

### 18.12 Protection-FRT consequence classes

Extend the interface class to

$$
c=(c_D,c_I,c_P,c_{FRT},c_{rec},c_R),
$$

where detection, isolation, protection, ride-through, reconnection, and
restoration outcomes are jointly represented:

$$
\pi_c(k)=
\Pr\left(
C_D=c_D,C_I=c_I,C_P=c_P,C_{FRT}=c_{FRT},
C_{rec}=c_{rec},C_R=c_R
\mid K=k
\right).
$$

Let the pre-fault exposed DER set be

$$
\mathcal R_k=\{r:\chi_r(t_0^-)=1
\text{ and DER }r\text{ is electrically exposed to event }k\}
$$

This excludes DERs already disconnected before the initiating event. Define a
terminal classification time

$$
T_k^{term}=\min(T_k^{clear},T_k^{backup,max}),
$$

so a fail-to-clear case is still classified at the declared backup horizon.
For $\mathcal R_k\ne\varnothing$, define the FRT partition:

$$
\mathcal E_k^{FRT,trip}
=\{\exists r\in\mathcal R_k:\chi_r(T_k^{term})=0\},
$$

$$
\mathcal E_k^{FRT,MC}
=\{\chi_r(T_k^{term})=1\ \forall r\in\mathcal R_k\}
\cap\{\exists r\in\mathcal R_k:
\kappa_r^{FRT}(T_k^{term})=MC\},
$$

$$
\mathcal E_k^{FRT,ride}
=\{\chi_r(T_k^{term})=1\ \forall r\in\mathcal R_k\}
\cap\{\kappa_r^{FRT}(T_k^{term})\ne MC
\ \forall r\in\mathcal R_k\}.
$$

The trip-first ordering makes these three aggregate events mutually exclusive;
when $\mathcal R_k=\varnothing$, set $c_{FRT}=\mathrm{not\ applicable}$ rather
than relying on vacuous truth. For a nonempty exposed set, the three events are
collectively exhaustive only if every connected non-MC terminal state is mapped
to the ride/normal aggregate; device-specific extra states must be assigned
explicitly. Device-level states remain available when mixed fleet outcomes are
needed. The
aggregate labels are sufficient only when the DERs in $\mathcal R_k$ are
consequence-equivalent or the physical consequence engine also receives the
device-level trajectories $\{\chi_r(t),\kappa_r(t),a_r(t)\}_r$. One small PV
trip and one critical GFM-storage trip cannot be assigned the same consequence
merely because both satisfy $\mathcal E_k^{FRT,trip}$.

The table below shows representative cross-axis classes and class families.
Rows that omit an axis, such as the backup-cleared family, must still be crossed
with the applicable FRT and reconnection terminal values when constructing the
complete mutually exclusive class set:

| Class | Defining event | Consequence |
|---|---|---|
| Primary cleared, DER rides through | $\mathcal E_k^{pri}\cap\mathcal E_k^{FRT,ride}$ | smallest cleared zone; surviving DER available subject to current/SOC limits |
| Primary cleared, momentary cessation | $\mathcal E_k^{pri}\cap\mathcal E_k^{FRT,MC}$ | correct clearing but transient DER power deficit |
| Primary cleared, DER trip | $\mathcal E_k^{pri}\cap\mathcal E_k^{FRT,trip}$ | reconnection delay and ramp reduce restoration support |
| Backup cleared | terminal backup-cleared class derived from $\mathcal E_k^{bak}$ | expanded outage zone and longer clearing |
| Miscoordinated trip | $\mathcal F_k^{mis}$ | healthy zone or source is disconnected |
| Fail to clear | terminal fail-to-clear class; breaker failure may be a cause attribute | upstream clearing beyond the modeled horizon, damage escalation, longer repair |
| Transient self-cleared | fault extinguishes before any protection operation | no sustained interruption unless the disturbance caused DER cessation/trip or another device operation |
| Reclose blocked | $\neg\mathcal S_z^{sync}\lor\neg\mathcal S_z^{dead}$ | extended outage until manual or synchronized restoration |
| Island protection invalid | $\vartheta_z\ne\vartheta_z^*(\sigma)$ | islanded refusal/misoperation class |

The class-conditioned physical consequence is

$$
S_{k,c}(t)=
S\left(
\Phi_{k,c_P,c_{FRT},c_I}(G_0),
\mathcal A_{c_R},
\chi(t),
\overline p^{avail}(t)
\right).
$$

The reliability contribution must integrate the trajectory:

$$
\mathrm{EENS}_{k,c}
=\frac{\lambda_k\pi_c(k)}{3600}
\int_0^{T_{k,c}^{end}}S_{k,c}(t)\,dt.
$$

Here event time is in seconds and shed is in MW; omit the factor $1/3600$ only
when $t$ is already measured in hours. A piecewise-constant staged
approximation may replace the integral only after the protection/FRT event
sequence defines the stage boundaries.

#### Computable protection-FRT class generator

Let $\omega\in\mathcal W$ collect the pre-fault operating point, load and DER
output, topology and setting group, cyber/auxiliary-power state, weather,
fault parameters, and uncertain device/FRT parameters. The proposed executable
interface is

$$
\mathcal M_{PFRT}:(k,\omega)\mapsto
\left(
C_P,C_{FRT},C_{rec},T^{clear},\Phi_{k,c},
\{\chi_r(t),\kappa_r(t),a_r(t)\}_r,
\mathcal V^{PFRT}
\right),
$$

where $\mathcal V^{PFRT}$ contains fidelity, convergence, parameter-provenance,
and validity flags. Detection, isolation, and restoration models then augment
this output to the complete class $c$. For fixed $k$ and $\omega$, the terminal
classification is deterministic unless an internal stochastic branch is
explicitly sampled.

The joint class probability is

$$
\pi_c(k)=
\int_{\mathcal W}
\mathbf 1[\mathcal C(k,\omega)=c]\,
f_{\omega\mid K}(\omega\mid k)\,d\omega.
$$

For iid conditional samples $\omega_n\sim f_{\omega\mid K}(\cdot\mid k)$,

$$
\widehat\pi_c(k)=\frac1N\sum_{n=1}^{N}
\mathbf 1[\mathcal C(k,\omega_n)=c].
$$

For quadrature, stratified scenarios, or importance sampling with normalized
weights $w_n\ge0$ and $\sum_nw_n=1$,

$$
\widehat\pi_c(k)=\sum_{n=1}^{N}w_n
\mathbf 1[\mathcal C(k,\omega_n)=c].
$$

Empirical replay, calibrated event-tree branches, dynamic ensembles, and
validated surrogate classifiers are alternative estimators of the same
conditional probability. The study must report the conditioning distribution,
sample/weight construction, confidence interval, unresolved class mass, and
surrogate error near pickup, FRT, current-limit, and synchronization boundaries.

#### Reduction of event trajectories to reliability stages

Let the event solver return ordered boundaries

$$
0=t_0<t_1<\cdots<t_J=T_{k,c}^{end},
$$

including DER mode changes, relay commands, breaker openings, fault extinction,
reclose/lockout, reconnect qualification, and restoration actions. On interval
$j$, construct a consequence state

$$
\Xi_{k,c,j}=
\left(G_{k,c,j},\overline p_{r,j}^{avail},
\kappa_{r,j},E_j^{st},\mathcal A_{c_R,j}\right),
$$

and solve

$$
S_{k,c,j}=S(\Xi_{k,c,j}),\qquad
\tau_{k,c,j}^{hr}=\frac{t_j-t_{j-1}}{3600}
$$

when event time is measured in seconds. The reliability contribution becomes

$$
\mathrm{EENS}_{k,c}
\approx
\lambda_k\pi_c(k)
\sum_{j=1}^{J}S_{k,c,j}\tau_{k,c,j}^{hr}.
$$

Storage state and other intertemporal resources must be advanced in the same
order; independently solving each interval from the initial SOC is invalid.
Adjacent intervals may be merged only when topology, DER/control availability,
resource state, and the physical consequence are equivalent at the required
fidelity. Protection/FRT boundaries are therefore allowed to refine the
three-stage model into a multistage consequence sequence. Sub-cycle waveform
effects that cannot be represented by the phasor/staged engine remain a P3/EMT
validation issue; momentary and sustained customer-interruption indices must
also retain their applicable duration definitions.

### 18.13 IEEE-1547 parameter interface

IEEE 1547 is an interconnection/performance standard, not a stochastic
reliability model. For each DER, the reliability interface needs:

| Parameter | Meaning |
|---|---|
| $id_r^{std}$ | applicable standard version/amendment and abnormal-operating-performance category |
| $id_r^{profile}$ | utility/interconnection profile identifier and revision |
| $\mathcal J_{r,V},\mathcal J_{r,f}$ | configured voltage/frequency bands and band semantics |
| $T_{r,j}^{trip}$ | actual band clearing/trip times |
| $\mathcal M_r^{cessation}$ | momentary-cessation entry, output, and exit behavior |
| $I_r^{max}$ | converter current limit on a stated base |
| $\pi_r^{priority}$ | magnitude, active, reactive, hybrid, or saturation priority |
| $K_{q,r}$ | voltage-reactive-current response with deadband/filter/ramp |
| $\mathcal V_r^{rec},\mathcal F_r^{rec}$ | reconnect qualification windows |
| $T_r^{stable},T_r^{close},T_r^{ramp}$ | reconnect dwell, closing, and power-ramp times |
| $\mathcal S_r^{sync}$ | synchronization requirement and tolerances |
| $\eta_r^{evidence}$ | category, utility setting revision, firmware/device evidence, and uncertainty |

Actual behavior should be represented as

$$
\eta_r^{FRT}
=\mathcal P\left(
\mathrm{standard\ version/category},
\mathrm{interconnection\ profile},
\mathrm{utility\ settings},
\mathrm{manufacturer\ implementation},
\mathrm{firmware},
\mathrm{test\ evidence}
\right),
$$

not as a standard name alone. If a default curve is used, provenance must be
template/assumption. In particular,

$$
\text{IEEE 1547 compliance}
\not\Rightarrow
\text{DER remains available in every contingency}.
$$

Connection state is disturbance-dependent:

$$
\chi_r(t)=
\mathcal F_{1547,r}\left(
\{V_r(\tau),f_r(\tau)\}_{0\le\tau\le t},
\eta_r^{FRT}
\right),
$$

not simply the static equipment state $x_r^p$.

### 18.14 Protection parameter interface

Required data include:

- installed relay functions and protected/backup zones;
- CT/PT ratios, errors, saturation model, sampling/filtering, and polarity;
- pickup values, curve family, TMS/time dial, instantaneous elements, reset
  law, and coordination margin;
- directional polarization, distance zones, differential restraint, frequency,
  ROCOF, voltage, and anti-islanding settings actually enabled;
- trip-channel, auxiliary-power, trip-coil, breaker opening, arc-clearing, and
  breaker-failure timers;
- fuse minimum-melt/total-clear curves, recloser shot curves/dead times/lockout,
  and sectionalizer count logic;
- transient/permanent fault classification and reclose success evidence;
- setting-group selection logic for grid-connected and islanded modes;
- cyber bindings only for protection functions that actually use networked
  measurements, transfer trip, or remote settings.

Parameter provenance must distinguish field settings, coordination study,
manufacturer data, injection test, HIL test, template, and assumption.

### 18.15 Fidelity ladder and recommended integration

| Level | Protection/FRT model | Use |
|---|---|---|
| P0 | external $p^{FT}$, nuisance-trip rate, and deterministic zone expansion | current reliability screening |
| P1 | detailed short-circuit snapshot plus static pickup/direction checks | fast fault-by-fault protection reach screen |
| P2 | event-driven relay operating quantities, breaker/recloser sequence, DER band timers, current limit, trip/reconnect classes | recommended reliability target |
| P3 | full phasor-domain/DAE or EMT trajectory with unbalanced controls, CT saturation, detailed PLL/GFM limiting, communication timing, and HIL validation | critical feeders and protection design |

P1 can screen initial pickup feasibility, directional sign, static reach, and
gross coordination risk at a fixed fault snapshot. It cannot establish an
inverse-time accumulated operation under changing current, FRT timer outcome,
momentary-cessation timing, multi-shot reclose/lockout sequence, DER trip, or
reconnection delay. Those claims require at least P2 event evolution.

P2 should be the default target for intelligent reliability. It reuses current
short-circuit and dynamics kernels while avoiding the cost of running full EMT
for every Monte Carlo hour. A practical workflow is:

1. precompute fault/protection/FRT classes for representative topology,
   operating, DER, and cyber states;
2. validate critical boundaries with the dynamic solver;
3. store class probabilities, clearing times, DER trajectories, and validity;
4. reuse these certified classes in FMEA/three-stage/Monte Carlo consequence
   evaluation.

Interpolation or surrogate classification must preserve conservative boundary
handling near pickup, trip-curve, current-limit, and synchronization surfaces.

### 18.16 Required validity flags

Add the following result capabilities:

- protection\_logic\_modelled;
- protection\_time\_current\_curves\_modelled;
- directional\_protection\_modelled;
- distance\_or\_differential\_protection\_modelled;
- breaker\_failure\_logic\_modelled;
- recloser\_fuse\_sectionalizer\_sequence\_modelled;
- reclosing\_and\_lockout\_modelled;
- ieee1547\_ride\_through\_modelled;
- der\_momentary\_cessation\_modelled;
- der\_fault\_current\_limit\_modelled;
- der\_reconnection\_delay\_modelled;
- adaptive\_protection\_settings\_modelled;
- islanded\_protection\_validated;
- short\_circuit\_model\_available;
- unbalanced\_fault\_and\_sequence\_controls\_modelled;
- protection\_frt\_reliability\_coupled.

These are capability statuses, not automatically inferred feature names.
Functions absent from the installed scheme should be `not_applicable`; installed
functions omitted by the analysis should be `not_modelled`.

For the current reliability evaluators, these flags would remain false except
for coarse protection-zone/interlock abstractions. A separate dynamics run does
not make a reliability result FRT-aware unless its event outcomes are actually
coupled into that result. In particular,
`protection_frt_reliability_coupled=true` requires a traceable class-generator
or trajectory-to-stage record for the reported metric; it cannot be inferred
from `short_circuit_model_available` or
`ieee1547_ride_through_modelled` alone.

## 19. Physical microgrid and DER reliability model

### 19.1 Island feasibility

**Proposed extension using existing physical kernels.** A microgrid island $q$
is serviceable only if it has an available grid-forming anchor:

$$
y_q^{island}\le
\sum_{r\in\mathcal R_q^{GFM}}x_r^p\phi_{ctrl(r)}(x^c).
$$

With binary $y_q^{island}$ and binary anchor-availability products, this
already enforces that at least one anchor is available; no explicit
$\min(1,\cdot)$ is needed. A linear MILP implementation must introduce an
auxiliary binary for each product $x_r^p\phi_{ctrl(r)}$.

Active and reactive adequacy require

$$
\sum_{r\in\mathcal R_q}p_r^{DER}+p_q^{st}
+p_q^{sh}=P_q^d+P_q^{loss},
$$

$$
\sum_{r\in\mathcal R_q}q_r^{DER}+q_q^{sh}=Q_q^d+Q_q^{loss}.
$$

Converter capability should use

$$
(p_r^{DER})^2+(q_r^{DER})^2\le(S_r^{rated})^2,
$$

or a documented polyhedral approximation. A grid-forming flag without rating,
energy, reactive capability, and control-channel availability is insufficient
evidence that the island can be sustained.

This is a second-order-cone constraint. An SOCP solver may enforce it directly.
An LP/MILP must state whether its polygon is an inner conservative
approximation or an outer optimistic relaxation. For equally spaced facet
normals, one conservative inner approximation is

$$
p_r\cos\theta_j+q_r\sin\theta_j
\le S_r^{rated}\cos(\pi/J),\qquad j=1,\ldots,J.
$$

### 19.2 Storage endurance

For chronological assessment,

$$
E_{b,t+1}=E_{b,t}
+\eta_b^{ch}p_{b,t}^{ch}\Delta t
-\frac{p_{b,t}^{dis}}{\eta_b^{dis}}\Delta t,
$$

$$
\underline E_b\le E_{b,t}\le\overline E_b,
\quad 0\le p_{b,t}^{ch}\le\overline P_b^{ch},
\quad 0\le p_{b,t}^{dis}\le\overline P_b^{dis}.
$$

Unless simultaneous charge/discharge is deliberately allowed, add

$$
0\le p_{b,t}^{ch}\le u_{b,t}\overline P_b^{ch},
$$

$$
0\le p_{b,t}^{dis}\le(1-u_{b,t})\overline P_b^{dis},
\qquad u_{b,t}\in\{0,1\}.
$$

Storage must not be independently credited at full power in every restoration
stage or overlapping outage. Event-stage energy bookkeeping is implemented in
the three-stage model; multi-event chronology requires sequential simulation.

### 19.3 Renewable uncertainty and DER control modes

Available renewable power is

$$
0\le p_{r,t}\le x_{r,t}^p\bar p_{r,t}^{weather}.
$$

If communication/control is lost, use the actual fallback mode:

$$
p_{r,t}=
\begin{cases}
p_{r,t}^{last}, & \text{setpoint hold},\\
p_{r,t}^{local}(v,f), & \text{local droop/autonomous control},\\
0, & \text{fail-safe trip},
\end{cases}
$$

rather than treating every communication loss as a forced outage. These three
fallback policies can have very different reliability value.

### 19.4 Synchronization and reconnection

Before closing a tie between energized islands,

$$
|\Delta V|\le\Delta V^{max},\quad
|\Delta f|\le\Delta f^{max},\quad
|\Delta\theta|\le\Delta\theta^{max}.
$$

These constraints are important for intelligent restoration with multiple
grid-forming DERs. A topology-only restoration plan that ignores synchronization
must be labeled as an upper bound on achievable service.

## 20. Physical-to-cyber back-coupling

**Proposed for chronological Level 3.** A cyber node $u$ is powered by physical
bus $\pi(u)$. With backup energy $B_u$,

$$
B_{u,t+1}=\min\left(\bar B_u,
B_{u,t}+P_{u,t}^{charge}\Delta t-P_u^{load}\Delta t\right).
$$

The upper-clamp-only expression is incomplete if the battery depletes inside
the step. The physically bounded update is

$$
B_{u,t+1}=
\min\left(
\bar B_u,\,
\max\left(0,\,
B_{u,t}+P_{u,t}^{charge}\Delta t-P_u^{load}\Delta t
\right)
\right).
$$

Its availability is

$$
x_{u,t}^c=x_{u,t}^{intrinsic}
\mathbf 1[y_{\pi(u),t}=1\ \lor\ B_{u,t}>0].
$$

After power returns, the node may remain unavailable for reboot and link
re-establishment time $T_u^{boot}$. This feedback can create delayed loss of
visibility during a long physical outage. It should be simulated on a timeline;
folding it into one static cyber availability obscures battery endurance and
restoration order.

Moreover, $B_{u,t}>0$ does not imply service for the entire next time step.
Remaining ride-through is

$$
T_u^{remain}=\frac{B_{u,t}}{P_u^{load}}.
$$

If $T_u^{remain}<\Delta t$, the node fails inside the interval. An event queue
or fractional-duration accounting is required; an end-of-step binary update
will misstate function availability.

## 21. Unified integrated reliability equation

### 21.1 Analytical interface-matrix form

Partition all cyber/intelligent states that have the same physical behavior
into classes $c\in\mathcal C(k)$. For each initiating physical/protection event
$k$,

$$
\boxed{
\mathrm{EENS}=
\sum_k\lambda_k
\sum_{c\in\mathcal C(k)}\pi_c(k)
\sum_{s\in\mathcal S(k,c)}
S_{k,c,s}\tau_{k,c,s}.}
$$

Here

$$
\pi_c(k)=\Pr(C=c\mid K=k),\qquad
\pi_c(k)\ge0,\qquad
\sum_{c\in\mathcal C(k)}\pi_c(k)=1.
$$

Classes must be mutually exclusive and collectively exhaustive over all
detection, isolation, restoration, protection, and cyber outcomes. Here
$\pi_c(k)$ is produced by detection, isolation, restoration, protection,
and cyber-graph models; $S_{k,c,s}$ is produced by the physical consequence
engine after applying class-specific patches; and $\tau_{k,c,s}$ is produced by
automatic/manual action timing and repair models.

For DER/protection-aware assessment, use the explicit tuple

$$
c=(c_D,c_I,c_P,c_{FRT},c_{rec},c_R),
$$

with joint probability

$$
\pi_c(k)=
\Pr\left(
C_D=c_D,C_I=c_I,C_P=c_P,C_{FRT}=c_{FRT},
C_{rec}=c_{rec},C_R=c_R
\mid K=k
\right).
$$

Protection and DER terms must not be multiplied as independent marginals when
they share voltage, current, clearing time, communication, or auxiliary-power
states. The class consequence consumes both the topology mutation and the
time-varying DER availability produced by Section 18.

If duration or shed is continuously random inside a class, the term
$S_{k,c,s}\tau_{k,c,s}$ must be replaced by its joint conditional expectation:

$$
\mathbb E\left[
\int_{\mathcal T_{k,c,s}}S_{k,c}(t)\,dt
\;\middle|\;K=k,C=c
\right].
$$

The matching indices are

$$
\mathrm{LOLE}=\sum_k\lambda_k\sum_c\pi_c(k)
\sum_s\tau_{k,c,s}\mathbf 1[S_{k,c,s}>\varepsilon],
$$

$$
\mathrm{LOLF}=\sum_k\lambda_k\sum_c\pi_c(k)
\mathbf 1[\exists s:S_{k,c,s}>\varepsilon].
$$

This analytical meaning counts initiating contingencies that cause any load
loss, not separate loss episodes. If restoration and later depletion can create
multiple episodes, define

$$
N_{k,c}^{loss}
=\sum_s\mathbf 1\left[
S_{k,c,s}>\varepsilon
\land
(s=1\ \lor\ S_{k,c,s-1}\le\varepsilon)
\right],
$$

and use

$$
\mathrm{LOLF}
=\sum_k\lambda_k\sum_c\pi_c(k)N_{k,c}^{loss}.
$$

### 21.2 Chronological form

For dynamic coupling, simulate joint events and decisions:

$$
\xi_{t+\Delta t}=F(\xi_t,w_t,a_t),
\qquad a_t=\pi_{FDIRP}(\hat x_t),
$$

$$
\mathrm{EENS}_y=\int_0^H S(\xi_t,a_t)\,dt.
$$

This form captures common weather hazards, cyber backup depletion, delayed
reboot, overlapping repairs, storage depletion, crew queues, and annual tail
risk. It should reuse the existing sequential Monte Carlo framework with an
event queue and sub-hour timestamps rather than introduce a separate
co-simulation architecture.

## 22. Common-cause and dependency modeling

Independent Bernoulli products are insufficient for the principal risks of an
intelligent cyber-physical system. At minimum, model:

- one weather process that changes line, wireless-link, and site-power hazard;
- shared communication conduits and switches;
- common firmware/configuration versions across IED fleets;
- one control-center or time-synchronization dependency;
- common auxiliary DC supply at each substation;
- protection/automation functions sharing the same measurements;
- DER fleets sharing an aggregator or cloud control plane.

A tractable beta-factor common-cause model for redundant elements is

$$
\lambda_q^{ind}=(1-\beta_q)\lambda_q,
\qquad
\lambda_g^{cc}=\beta_q\lambda_q,
$$

where group event $g$ fails all members. For weather, use time-varying hazards

$$
\lambda_m(t)=\lambda_m^0\,h_m(W_t),
$$

with the same $W_t$ driving physical and communication assets. This shared
driver is preferable to an arbitrary correlation coefficient because it has an
observable engineering cause.

For a homogeneous redundant group, $\lambda_g^{cc}=\beta\lambda$ is **one**
shared event frequency that fails the modeled group. It must not be added once
per member, which would incorrectly create $n\beta\lambda$. Each member retains
its independent frequency $(1-\beta)\lambda$. Heterogeneous groups require an
explicit common-cause event set rather than one beta parameter.

If $\lambda_m(t)$ is the intensity of a non-homogeneous Poisson process, then

$$
\Pr[N_m(t,t+\Delta t)\ge1]
=1-\exp\left(
-\int_t^{t+\Delta t}\lambda_m(s)\,ds
\right).
$$

This integral, not $\lambda_m(t)\Delta t$ except as a small-step
approximation, defines event probability in the chronological model.

## 23. Extended metrics for engineering decisions

Existing EENS, LOLE, LOLF, SAIFI, SAIDI, CAIDI, ASAI, VaR, and CVaR remain the
primary reliability outcomes. Add decompositions rather than replace them:

$$
\mathrm{EENS}=\mathrm{EENS}^{physical}
+\Delta^{detection}+\Delta^{isolation}
+\Delta^{restoration}+\Delta^{protection}
+\Delta^{DER-FRT}
+\Delta^{cyber\ power}.
$$

Useful supporting measures are:

1. **FDIR success probability** per contingency:
   $P_{FDIR}(k)=\Pr(c=\text{successful automatic restoration}\mid k)$.
2. **Expected detection/isolation/restoration latency** and its 95th percentile.
3. **Correct isolation rate** and **backup-zone operation rate**.
4. **Protection dependability/security**: fail-to-trip per demand and nuisance
   trips/year.
5. **Cyber-caused SAIDI share** relative to perfect cyber.
6. **Automation efficacy** relative to perfect/no-automation bounds.
7. **Microgrid survival probability** for horizons 1, 4, 8, and 24 hours.
8. **Critical cyber cut-set importance**, calculated by marginal re-evaluation,
   not simple co-occurrence.
9. **DER autonomy value**: EENS difference between local fallback control and
   communication-dependent trip/hold behavior.
10. **Primary/backup/selectivity rates**: conditional probabilities of primary
    clearing, backup clearing, miscoordination, breaker failure, and fail to
    clear per physical fault class.
11. **DER disturbance-state shares**: ride-through, momentary cessation, trip,
    blocked reclose, reconnect-wait, and ramping probabilities.
12. **Protection-FRT EENS increment**: difference between dynamic
    protection/FRT-conditioned EENS and the static always-available DER
    counterfactual.

For investment ranking, use marginal value

$$
V_q=\frac{\mathrm{EENS}_{base}-\mathrm{EENS}_{upgrade(q)}}
{\mathrm{annualized\ cost}_q},
$$

and report confidence intervals or parameter ranges. Do not rank upgrades by
cyber availability alone; a highly unavailable service can have low physical
consequence, while a rare shared cut set can dominate system EENS.

## 24. Required data model

### 24.1 Physical data

- domain-qualified buses and topology;
- normal and restoration switch states, roles, and interrupting capabilities;
- protection zones, primary/backup relationships, setting groups, and action
  times;
- protection pickup/curve/reset settings, breaker-failure/reclose logic,
  CT/PT evidence, and fault-zone bindings from Section 18.14;
- component failure frequencies, repair distributions, and common-cause groups;
- DER active/reactive ratings, grid-forming capability, local fallback mode,
  black-start capability, and availability;
- DER ride-through bands, cessation policy, current limit/priority, reconnect
  qualification, synchronization, and ramp evidence from Section 18.13;
- storage power, energy, SOC limits, efficiencies, and initial SOC;
- microgrid membership, PCC, islanding capability, and synchronization limits;
- load priority, customer counts, critical-load class, and profiles.

### 24.2 Cyber data

- nodes: sensor/IED/FTU/RTU, switch/router, gateway, server, controller, control
  center, clock, and power supply;
- links: endpoints, medium, route/conduit, latency distribution, packet loss,
  bandwidth class, failure and repair data;
- redundancy groups and shared-risk link groups;
- physical power-bus binding, battery endurance, and reboot time;
- service bindings from sensor through application to actuator;
- local/manual fallback capability for every remotely controlled action.

### 24.3 Intelligent-function data

- detector confusion matrix by fault type and observability class;
- detection and location latency distributions;
- isolation-zone confusion or primary/backup-zone probabilities;
- restoration policy version, feasible action set, safety-filter outcomes, and
  execution success;
- protection fail-to-trip probability per demand, nuisance-trip frequency, and
  adaptive-setting failure data;
- conditional operating-state distribution $f_{\omega\mid K}(\omega\mid k)$
  or an auditable weighted scenario design for protection-FRT class estimation;
- class-generator version, unresolved-class policy, surrogate training domain,
  boundary error, and probability confidence intervals;
- provenance: field record, vendor test, HIL test, simulation, template, or
  assumption.

Templates are acceptable for screening but must remain visibly tagged. A
decision-grade study needs utility event logs and communication/service data.

## 25. Calibration, uncertainty, and validation

### 25.1 Parameter uncertainty

Sparse cyber and intelligent failure data should be represented with
distributions. For example,

$$
p_D\sim\mathrm{Beta}(\alpha_D,\beta_D), \qquad
\lambda\sim\mathrm{Gamma}(\alpha_\lambda,\beta_\lambda).
$$

Propagate epistemic uncertainty outside the aleatory reliability simulation:
sample parameter sets, run the reliability model, and report credible intervals
for EENS/SAIDI and upgrade value. Do not collapse poorly known cyber risk into
false decimal precision.

### 25.2 Validation hierarchy

1. Unit tests for two-state conversions, structure functions, cut sets, and
   class probabilities.
2. Small exact Markov models as oracles for two to six components.
3. Fault replay against historical detector/isolation outcomes.
4. Communication emulation or HIL validation of latency and loss contracts.
5. Protection-in-the-loop tests for primary/backup zone selection.
6. Secondary-injection/time-current tests for pickup, inverse-time integration,
   reset, direction, breaker-failure, and reclose/lockout boundaries.
7. DER FRT boundary tests immediately below/above each configured voltage,
   frequency, clearing-time, current-limit, cessation, and reconnect guard.
8. Cross-check the initial dynamic fault current against detailed
   short-circuit converter contributions for the same topology and device base.
9. Closed-loop tests in which DER current limit/trip changes relay operation
   order, clearing changes DER survival, and the final consequence class is
   deterministic and auditable.
10. Class-estimator checks for mutual exclusivity, unit total probability,
    confidence coverage, weighted-sample normalization, and unresolved mass.
11. Trajectory-to-stage checks that preserve event order, topology, DER
    availability, storage SOC, seconds-to-hours conversion, and integrated
    energy against direct time integration.
12. Restoration action replay against the same physical constraints used in the
   reliability consequence engine.
13. End-to-end reference feeders with AC/DC, microgrid islanding, storage
   endurance, cyber-node backup power, and common weather hazards.
14. Cross-method checks: analytical class-conditioned FMEA should agree with
   long-run sequential Monte Carlo when its assumptions are reproduced.

For two independent repairable components, a useful exact CTMC oracle has
states $00,10,01,11$, where 1 denotes down. With hourly failure intensities
$\alpha_i$ and repair intensities $\beta_i$, one consistent state ordering has

$$
Q=
\begin{pmatrix}
-(\alpha_1+\alpha_2) & \alpha_1 & \alpha_2 & 0\\
\beta_1 & -(\beta_1+\alpha_2) & 0 & \alpha_2\\
\beta_2 & 0 & -(\alpha_1+\beta_2) & \alpha_1\\
0 & \beta_2 & \beta_1 & -(\beta_1+\beta_2)
\end{pmatrix}.
$$

Solve

$$
\pi Q=0,\qquad \sum_x\pi_x=1,
$$

then compare

$$
\mathrm{EDNS}_{exact}=\sum_x\pi_xS(x)
$$

against non-sequential Monte Carlo and the first/second-order expansion. This
oracle verifies probability normalization, pair-state weighting, and the
consequence map without sampling error. It should remain small; state-space
growth is exponential.

### 25.3 Required honesty flags

Proposed results should add:

- `cyber_topology_modelled`;
- `communication_qos_modelled`;
- `detection_uncertainty_modelled`;
- `protection_selectivity_modelled`;
- `restoration_duration_cyber_conditioned`;
- `cyber_power_coupling_modelled`;
- `der_dynamic_stability_certified`;
- `common_cause_modelled`;
- `subhour_event_timing`.

All applicable protection/FRT capability statuses in Section 18.16 are
additionally required for a result that claims protection-logic or
IEEE-1547-aware reliability. A three-valued status
`modelled`/`not_modelled`/`not_applicable` is preferable to booleans: an
installed-but-omitted distance element is different from a feeder on which no
distance element exists.

A result must not imply full intelligent cyber-physical coverage when these are
false.

## 26. Recommended implementation sequence

### Phase 0: consolidate the physical baseline

1. Keep all methods on the shared reliability resolver.
2. Preserve current repair-window semantics and N-0 incremental qualification.
3. Surface model scope and fallback flags beside every reported metric.
4. Treat optional N-2 failure-mode totals as exposure approximations unless
   converted to a disjoint-state or interaction formulation.

### Phase 1: strengthen the shipped Level 1

1. Add per-function availability overrides for detection, isolation,
   restoration, and protection instead of one automation scalar.
2. Add three classes: full automation; observation available but DER control
   unavailable; manual/blocked automation.
3. Connect protection fail-to-trip demand frequency to the physical initiating
   faults in its zone.
4. Report the full EENS/SAIDI functional decomposition and perfect/no-automation
   bounds.

This phase answers whether deeper cyber modeling can materially change decisions
for a given feeder before expensive data collection begins.

### Phase 2: explicit cyber graph and function engine

1. Add `CyberSystem` case data for nodes, links, shared risks, power bindings,
   and service bindings.
2. Implement function structure evaluation and minimal cut-set/BDD or Monte
   Carlo calculation.
3. Produce consequence-equivalent class probabilities $\pi_c(k)$.
4. Reuse the existing consequence patches and physical engines once per class.
5. Add cyber-element marginal importance and N-1 cyber contingency ranking.

This is the recommended target architecture because it captures shared
communication dependencies without coupling two optimization solvers.

### Phase 2P: protection-FRT event interface

1. Map rich protection zones, switch/recloser/fuse metadata, short-circuit
   contributions, and dynamic DER settings into the Section 18 parameter
   contracts.
2. Implement P1 static pickup/direction/coordination screening.
3. Implement P2 relay operating quantities, breaker/reclose events, per-band
   DER FRT timers, current limiting, cessation, trip, reconnect, and ramp.
4. Generate mutually exclusive protection-FRT consequence classes with
   clearing time, topology mutation, DER availability trajectory, provenance,
   and validity flags.
5. Estimate $\pi_c(k)$ from a declared conditional operating/cyber/device
   distribution, with confidence intervals and unresolved-class accounting.
6. Reduce each accepted event trajectory to an ordered multistage consequence
   sequence while preserving DER availability and storage chronology.
7. Validate critical class boundaries with the existing dynamic solver before
   allowing class reuse in FMEA, three-stage restoration, or Monte Carlo.

This phase is required before high-DER reliability results can claim protection
selectivity or IEEE-1547 ride-through coverage.

### Phase 3: chronological intelligent cyber-physical simulation

1. Upgrade sequential Monte Carlo to an event queue with minute/second event
   timestamps inside hourly load intervals.
2. Simulate cyber component sojourns, link QoS, physical faults, detection,
   protection/FRT classes or live P2 events, restoration actions, repair, and
   reboot events.
3. Add cyber backup-power timers and storage/DER energy chronology.
4. Drive physical and communication hazards from the same weather process.
5. Produce valid annual tail risk and storm-year distributions.

### Separate resilience/security track

Strategic attacks do not have a defensible Poisson failure frequency. Evaluate
them as scenario-conditional or attacker-defender resilience problems:

$$
\max_{a^{atk}:\|a^{atk}\|\le K}
\min_{a^{rest}} S(a^{atk},a^{rest}),
$$

and report worst-case shed, recovery time, and N-k cyber survivability beside,
not inside, ordinary EENS/SAIDI.

## 27. Experiment matrix for a comprehensive study

Run at least the following cases on representative AC, DC, and hybrid feeders:

| Axis | Cases |
|---|---|
| Automation | perfect; measured; no automation |
| Detection | nominal; missed-fault stress; high false-alarm stress |
| Isolation | primary zone; backup-zone probability; stuck sectionalizer |
| Restoration | automatic FLISR; partial comms; manual crew; blocked tie |
| Protection | nominal; fail-to-trip; nuisance trip; adaptive-setting failure |
| Fault/protection type | three-phase; SLG; LL; LLG; high-impedance; primary/backup/breaker-failure |
| Protection coordination | correct margin; underreach; overreach; reverse flow; CT/PT error; recloser/fuse/sectionalizer sequence |
| DER FRT | mandatory ride-through; permissive/setting uncertainty; momentary cessation; trip; reconnect blocked |
| Inverter fault response | GFL current source; GFM voltage source; magnitude/active/reactive-priority limiting; PLL/control stress |
| Reconnection | immediate counterfactual; qualified delay; sync-check; lockout; ramp-limited return |
| Cyber topology | radial comms; redundant ring; shared control-center outage |
| Cyber power | infinite backup; measured battery; extended physical outage |
| DER behavior | local autonomous fallback; frozen setpoint; fail-safe trip |
| Microgrid | no islanding; GFM islanding; GFM/control loss; storage depletion |
| Dependency | independent; common weather; shared firmware/configuration |
| Method | class-conditioned FMEA; three-stage MILP; sequential MC |

Report not only total EENS but the incremental value of detection, isolation,
restoration, protection, communication redundancy, local DER autonomy, and
microgrid endurance. This separates an attractive automation feature from an
actual reliability improvement.

## 28. Overall assessment

The present reliability module has a strong reusable base: current parameter
semantics are unified; Monte Carlo supports hybrid consequences; staged FMEA
keeps switching and repair windows disjoint; the three-stage model holds the
fault and restoration plan through MTTR; storage and grid-forming support are
bounded; and model-scope flags disclose approximations.

Several implemented outputs still require explicit academic qualification:
failure-mode recovery duration has an unresolved start-time convention; N-2
pair consequences are added as exposure terms rather than a disjoint-state
correction; the AC restoration branch rating is an optimistic $P/Q$ box;
non-sequential annual tails assume iid hours; sequential MC rounds short events
to at least one hour; and empirical CVaR is a simple tail mean rather than the
strict discrete Expected Shortfall estimator. These are declared limitations,
not properties of the proposed ideal model.

The shipped Level-1+ cyber-physical model is useful for bounding the value of
automation, separating duration loss from DER/control loss, and screening
physical/information/intelligent parameters. Its intelligent factors remain a
conditional-independence product routed through two consequence classes. It is
not yet the full intelligent cyber-physical model because it cannot reveal
shared communication cut sets, detector confusion classes, protection
selectivity, cyber-node backup depletion, joint policy/action dependencies, or
protection-FRT trajectories.

The repository already has valuable building blocks beyond the reliability
module: opt-in device-level IEEE-1547 voltage/frequency timers, trip, reconnect,
and ramp logic in dynamics; GFL/GFM current-limited dynamic models; and detailed
short-circuit converter contributions. The limiting gap is their integration
with relay pickup/coordination, breaker/recloser event sequences, consequence
classes, and annual reliability weighting. Until that interface exists,
high-DER reliability results may be optimistic or conservative depending on
whether actual ride-through, cessation, trip, and protection interaction would
preserve or remove DER support.

The highest-value extension is not a monolithic co-simulator. It is an explicit
cyber/service graph that produces a small interface matrix of
consequence-equivalent classes, followed by reuse of the existing physical
restoration engines. Chronological simulation should then be added only for
phenomena that genuinely require time: sub-hour FDIR, common weather, storage
and cyber-battery depletion, reboot, repair overlap, crews, and annual tail
risk.

For intelligent functions, reliability credit must be based on validated
end-to-end performance: correct and timely detection, selective isolation,
safe restoration actions that actually execute, and dependable/secure
protection under both grid-connected and DER-dominated island conditions. This
keeps the assessment physically meaningful and prevents algorithm accuracy or
communication availability from being mistaken for delivered customer
reliability.

## 29. Source cross-reference

- Parameter, Monte Carlo, COPT, customer indices, component FMEA, Level-1 cyber
  conditioning, and physical consequence engines:
  `src/reliability/reliability_assessment.cpp`.
- Failure-mode catalog, consequence operator, protection-zone approximation,
  and N-2 pair enumeration: `src/reliability/failure_mode.cpp`.
- Three-stage AC/DC restoration model, storage chronology, protection/interlock
  checks, and N-0 qualification: `src/reliability/three_stage_reliability.cpp`.
- Device-level IEEE-1547 timers, trip/reconnect state, and smart-inverter
  functions: `include/hacdcpf/dynamics/devices/IEEE1547Protection.hpp` and
  `src/dynamics/devices/IEEE1547Protection.cpp`.
- GFL/GFM dynamic current limits and protection hooks:
  `src/dynamics/devices/BasicDynamicDevices.cpp` and
  `src/dynamics/DynamicSolver.cpp`.
- Detailed short-circuit converter contribution models:
  `include/hacdcpf/analysis/short_circuit.hpp` and
  `src/short_circuit/short_circuit.cpp`.
- Public contracts: `include/hacdcpf/reliability/reliability_assessment.hpp`,
  `include/hacdcpf/reliability/failure_mode.hpp`, and
  `include/hacdcpf/analysis/three_stage_reliability.hpp`.
- Restoration formulation context:
  [network_reconfiguration_models.md](network_reconfiguration_models.md).
- Existing focused references:
  [reliability_assessment_models.md](reliability_assessment_models.md) and
  archived [cyber-physical fidelity ladder](archive/theory/cyber_physical_reliability_extension.md).

Canonical external starting points include Billinton and Allan for power-system
reliability, IEEE 1366 for distribution indices, IEC 61850 for substation
communication, and IEEE 1547-2018 for DER interconnection and control. Specific
parameter values must come from the studied utility, vendor evidence, or
validated tests rather than these standards alone.

> Documentation Sync (2026-07-12)
> Scope: reviewed against current repository structure, CMake presets/options, and registered test targets.
> Status: implementation-backed reference.
> Source of truth: when text and implementation diverge, treat src/, include/, tests/, and CMake files as authoritative.

> Runtime companion: see `runtime_api.md`, `projection_and_results.md`, and
> `gui_canvas_runtime.md` for the current cached-frame and Canvas contracts.

# 时序生产模拟 — Rich-Model Mathematical Derivation (Review Draft)

> **Purpose.** This document (1) restates *exactly* the unit-commitment /
> economic-dispatch / power-flow models that the time-series production
> simulation **currently** implements so they can be double-checked, and
> (2) derives the **rich models** that extend coverage to **all** component
> types in the package.
>
> **Status (updated).** All nine rich models in §4 (§4.1–§4.9) are now
> **implemented** in `build_uc_milp`, each behind its own opt-in
> `TimeSeriesPFOptions` flag (default off, so the baseline solve is unchanged),
> exercised by a dedicated `tests/test_uc_*.cpp`, and exposed as a checkbox in
> the time-series GUI. Where the build deviates from the ideal derivation below
> the difference is called out in the relevant subsection (e.g. §4.1 uses a
> single net-exchange variable, §4.4 takes the relocation schedule as input,
> §4.9 uses an inflow/outflow split). Marks: 🆕 = rich model (now built).
>
> **Implementation anchors.**
> `src/time_series/time_series_pf.cpp` (`build_uc_milp`, `solve_unit_commitment`,
> `solve_time_series_pf`), `src/time_series/annual_production_sim.cpp`
> (`solve_annual_production_simulation`), `src/time_series/lifecycle_simulation.cpp`.
> Component data: `include/hacdcpf/model/{ac_components,dc_components,converter_components}.hpp`.

Legend: ✅ implemented · ⚠️ partial / approximated · 🆕 rich model (now implemented, opt-in) · ❌ not modelled.

---

## 1. Indices, sets, and data

| Symbol | Meaning |
|---|---|
| $t\in\mathcal T=\{0,\dots,T-1\}$ | time steps; step length $\Delta t$ (h) |
| $i\in\mathcal B^{ac}$, $n\in\mathcal B^{dc}$ | AC / DC buses |
| $g\in\mathcal G$ | dispatchable AC generators (`ac.generators`) |
| $s\in\mathcal S$, $d\in\mathcal S^{dc}$ | AC / DC storage (`ac.storage`, `dc.storage`) |
| $r\in\mathcal R$ | renewable generators (`ac.renewable_gens`) |
| $c\in\mathcal C$, $k\in\mathcal K$ | VSC / DC–DC converters (`vsc_converters`, `dc.dcdc_converters`) |
| $\ell\in\mathcal L^{ac}$, $m\in\mathcal L^{dc}$ | AC / DC branches |
| $x\in\mathcal X$ | external grids (`ac.external_grids`) |
| $f\in\mathcal F$ | flexible / DR loads (`ac.flexible_loads`) |
| $b\in\mathcal M$ | mobile storage (`mobile_storage`) |
| $v\in\mathcal V$, $\mu\in\mathcal U$ | VPPs / microgrids (`vpps`, `microgrids`) |

Per-unit on system base $S_{\text{base}}$. Generator cost coefficients
$c_2^g,c_1^g,c_0^g$ are `cost_c2/c1/c0`; $P^{\max}_g,P^{\min}_g$ are `pmax_mw/pmin_mw`;
ramp rates `ramp_up_mw_min`/`ramp_dn_mw_min` (MW·min⁻¹); `startup_cost`, `min_up_time_hr`,
`min_dn_time_hr`.

---

## 2. CURRENT model (as implemented — please verify)

### 2.1 Decision variables

$$
\begin{aligned}
&p_{g,t}\ge0,\ u_{g,t}\in\{0,1\},\ s^{\uparrow}_{g,t}\ge0 &&\text{(gen dispatch, commitment, startup cost)}\\
&p^{\text{ess}}_{s,t}\in[P^{\min}_s,P^{\max}_s],\ e_{s,t}\in[\underline{\text{soc}}_s,\overline{\text{soc}}_s] &&\text{(AC ESS power +=discharge, SOC)}\\
&p^{\text{dc,ess}}_{d,t},\ e^{dc}_{d,t} &&\text{(DC ESS power, SOC)}\\
&p^{\text{ren}}_{r,t}\in[0,\bar P^{\text{ren}}_{r,t}] &&\text{(renewable dispatch, curtailable)}\\
&\theta_{i,t} &&\text{(AC voltage angle, non-slack — network mode)}\\
&p^{\text{vsc}}_{c,t}\in[P^{\min}_c,P^{\max}_c],\ p^{\text{dcdc}}_{k,t} &&\text{(converter powers — DC-network mode)}
\end{aligned}
$$

### 2.2 Objective (minimise operating cost)

$$
\min\; \sum_{t}\Delta t\!\left[\sum_g\!\big(c_1^g\,p_{g,t}+c_0^g\,u_{g,t}\big)-\sum_r c^{\text{curt}}_r\,p^{\text{ren}}_{r,t}\right]+\sum_t\sum_g s^{\uparrow}_{g,t}.
\tag{2.1}
$$

The $-c^{\text{curt}}_r p^{\text{ren}}_{r,t}$ term rewards renewable utilisation (i.e.
penalises curtailment, `cost_curtail_mwh`). ⚠️ Note the generator cost is **linear**:
the quadratic `cost_c2` term is *not* in the MILP objective (it is recovered only in the
per-step AC-OPF stage).

### 2.3 Constraints

**(C1) System / nodal power balance.** Copper-plate (default):
$$
\sum_g p_{g,t}+\sum_s p^{\text{ess}}_{s,t}+\sum_d p^{\text{dc,ess}}_{d,t}+\sum_r p^{\text{ren}}_{r,t}=D_t,
\tag{C1a}
$$
where $D_t$ is the net system demand (AC + DC loads, minus must-take PV/static gen).
Network mode (`enable_network_constraints`, DC power flow) per AC bus $i$:
$$
\sum_{g\in i}p_{g,t}+\sum_{s\in i}p^{\text{ess}}_{s,t}+\sum_{r\in i}p^{\text{ren}}_{r,t}+\sum_{c\in i}p^{\text{vsc}}_{c,t}-S_{\text{base}}\!\!\sum_{j}B_{ij}\theta_{j,t}=D_{i,t}.
\tag{C1b}
$$

**(C2) DC nodal balance** (`enable_dc_network_constraints`) per DC bus $n$:
$$
\sum_{c\in n}\!\Big(\!-\tfrac{1}{\eta_c}p^{\text{vsc}}_{c,t}\Big)+\sum_{d\in n}p^{\text{dc,ess}}_{d,t}+\!\!\sum_{k:\text{out}(k)=n}\!\!\eta_k p^{\text{dcdc}}_{k,t}-\!\!\sum_{k:\text{in}(k)=n}\!\!p^{\text{dcdc}}_{k,t}=D^{dc}_{n,t}.
\tag{C2}
$$
⚠️ DC voltages are held flat ($V^{dc}\!\approx\!1$); DC branch flows are bounded only
*implicitly* through converter power limits (see §4.8 for the 🆕 explicit model).

**(C3) Generator output coupling.** $\;P^{\min}_g u_{g,t}\le p_{g,t}\le P^{\max}_g u_{g,t}.$

**(C4) Ramping.** $\;p_{g,t}-p_{g,t-1}\le 60\,R^{\uparrow}_g\Delta t,\quad p_{g,t-1}-p_{g,t}\le 60\,R^{\downarrow}_g\Delta t.$

**(C5) Startup cost linearisation.** $\;s^{\uparrow}_{g,t}\ge C^{\text{su}}_g\,(u_{g,t}-u_{g,t-1}),\ \ s^{\uparrow}_{g,t}\ge0$, with $u_{g,-1}$ from the initial dispatch state.

**(C6) Minimum up / down time.** For $k=1,\dots,\min(UT_g-1,T-1-t)$:
$$
u_{g,t}-u_{g,t-1}\le u_{g,t+k},\qquad u_{g,t-1}-u_{g,t}\le 1-u_{g,t+k}.
\tag{C6}
$$

**(C7) Storage SOC dynamics** (single power variable, $\eta_s=\sqrt{\eta^c_s\eta^d_s}$, self-discharge $\sigma_s$):
$$
e_{s,t}=(1-\sigma_s)\,e_{s,t-1}-\frac{\Delta t}{\eta_s E_s}\,p^{\text{ess}}_{s,t},\qquad e_{s,-1}:=\text{soc}^{\text{init}}_s.
\tag{C7}
$$
Identical form for DC storage $e^{dc}_{d,t}$.

**(C8) AC line thermal limits** (network mode): $\;\big|\tfrac{S_{\text{base}}}{x_\ell}(\theta_{i,t}-\theta_{j,t})\big|\le F^{\max}_\ell.$

**(C9) Spinning reserve** (`reserve_requirement_fraction` $\rho$): $\;\sum_g\big(P^{\max}_g u_{g,t}-p_{g,t}\big)\ge\rho\,D_t.$

### 2.4 Per-step AC-OPF + PF replay

After UC, each step optionally runs a full AC-OPF (`run_opf`) with a tracking band
$p^{\text{uc}}_{g,t}\pm\Delta$ around the UC dispatch, then a Newton power-flow validation
(`solve_time_series_pf`). The annual pipeline adds hierarchical temporal decomposition
(L0 annual → L1 monthly → L2 weekly UC → L3 daily UC+OPF/PF) with optional cyclic-SOC
boundary $e_{s,T-1}=e_{s,0}$.

---

## 3. Component coverage matrix

| Component (struct) | UC/ED status | Gap addressed in |
|---|---|---|
| `Generator` | ✅ full (UC, ramp, min up/dn, startup) | — |
| `Storage` (AC/DC), `DCStorage` | ✅ SOC dynamics | §4.7 (degradation cost 🆕) |
| `RenewableGen` | ✅ dispatch + curtailment | — |
| `ACBranch`, `Transformer2W` | ✅ DC-PF $B$-matrix + thermal | — |
| `VSCConverter`, `DCDCConverter` | ✅ (DC-network mode) + §4.8 flows/limits | **§4.8 🆕 done** |
| `Load`, `DCLoad`, `AsymmetricLoad` | ✅ fixed demand | — |
| `PVSystem`, `PVArrayDC`, `StaticGenerator(DC)` | ✅ §4.3 dispatchable (opt-in) | **§4.3 🆕 done** |
| `ExternalGrid` | ✅ §4.1 net-exchange (opt-in) | **§4.1 🆕 done** |
| `FlexibleLoad` | ✅ §4.2 demand response (opt-in) | **§4.2 🆕 done** |
| `ChargingStation`, `Charger` | ⚠️ fixed load | §4.2 / §4.6 🆕 (V2G) |
| `MobileStorage` | ✅ §4.4 (opt-in; exogenous schedule) | **§4.4 🆕 done** |
| `VirtualPowerPlant` | ✅ §4.5 (opt-in) | **§4.5 🆕** |
| `Microgrid` | ✅ §4.6 (opt-in) | **§4.6 🆕** |
| `EnergyRouter` | ✅ §4.9 (opt-in; else expanded→VSC/DC-DC) | **§4.9 🆕 done** |
| `DCBranch` thermal | ✅ §4.8 transport flows (opt-in) | **§4.8 🆕 done** |
| `Shunt`, `Transformer3W`, `RegulatorControl`, `AsynchronousMotor` | ❌ | §5 (AC-OPF layer, not UC) |
| `Switch`, `CircuitBreaker`, `DCCircuitBreaker` | ❌ topology fixed | §5 (reconfiguration module) |

---

## 4. 🆕 NEW rich-model derivations

All extensions stay **linear / mixed-integer-linear** so they fit the existing
MILP and remain solvable by Native B&C / HiGHS / SCIP.

**Build status (all opt-in, default off; each preserves the baseline solve).**

| § | Model | `TimeSeriesPFOptions` flag | Test | Build note |
|---|---|---|---|---|
| 4.1 | External grid | `enable_external_grid` | `test_uc_external_grid` | single net-exchange var (no buy/sell binary) |
| 4.2 | Demand response | `enable_demand_response` (+`dr_shiftable`) | `test_uc_demand_response` | up/down split; shiftable adds energy-neutrality row |
| 4.3 | Dispatchable PV / static gen | `enable_dispatchable_pv` | `test_uc_dispatchable_pv` | 4 source types (AC/DC PV, DC/AC static gen) |
| 4.4 | Mobile storage | `enable_mobile_storage` (+`mobile_storage_corelocate`) | `test_uc_mobile_storage` | exogenous schedule; opt-in co-optimised relocation (binaries) |
| 4.5 | Virtual power plant | `enable_vpp` | `test_uc_vpp` | aggregate output + energy envelope |
| 4.6 | Microgrid | `enable_microgrid` | `test_uc_microgrid` | PCC exchange + islanding binary |
| 4.7 | Storage degradation | `enable_storage_degradation` | `test_uc_storage_degradation` | throughput cost + daily-cycle cap |
| 4.8 | DC branch flows | `enable_dc_branch_flows` | `test_uc_dc_branch_flows` | transport flows + thermal limits |
| 4.9 | Energy router | `enable_energy_router` | `test_uc_energy_router` | per-port inflow/outflow + lossy conservation |

### 4.1 External grid — import/export with time-varying price 🆕

`ExternalGrid` carries `cost_c1`, `price_profile_id` (time-varying tariff
$\pi_{x,t}$), `emission_factor_tco2_mwh`, and `controllable`. Model each grid tie at
bus $i(x)$ as a bidirectional exchange split into non-negative import / export legs:

$$
p^{\text{imp}}_{x,t}\ge0,\quad p^{\text{exp}}_{x,t}\ge0,\quad y_{x,t}\in\{0,1\},
$$
$$
0\le p^{\text{imp}}_{x,t}\le \overline P^{\text{imp}}_x\,y_{x,t},\qquad
0\le p^{\text{exp}}_{x,t}\le \overline P^{\text{exp}}_x\,(1-y_{x,t}).
\tag{4.1a}
$$
The binary $y_{x,t}$ forbids simultaneous import/export (needed only when
$\pi^{\text{exp}}<\pi^{\text{imp}}$; otherwise it may be relaxed). Net injection
$p^{\text{ext}}_{x,t}=p^{\text{imp}}_{x,t}-p^{\text{exp}}_{x,t}$ enters the bus balance
(C1b). Objective adds purchase cost minus export revenue:
$$
\Delta J^{\text{ext}}=\sum_t\Delta t\sum_x\big(\pi^{\text{imp}}_{x,t}\,p^{\text{imp}}_{x,t}-\pi^{\text{exp}}_{x,t}\,p^{\text{exp}}_{x,t}\big),\quad
\pi^{\text{imp}}_{x,t}=\pi_{x,t}\,\text{(profile)}\ \text{or}\ c_1^x.
\tag{4.1b}
$$
Optional carbon term $\sum_t\Delta t\sum_x \kappa\,\varepsilon_x\,p^{\text{imp}}_{x,t}$ with
carbon price $\kappa$ and $\varepsilon_x=$ `emission_factor_tco2_mwh`. A ramp limit
$|p^{\text{ext}}_{x,t}-p^{\text{ext}}_{x,t-1}|\le R^{\text{ext}}_x\Delta t$ optionally
models tie-line schedule smoothness.

### 4.2 Flexible / demand-response loads 🆕

`FlexibleLoad` has baseline `p_mw` $P^0_{f,t}$, up/down headroom `flex_up_mw`
$\overline{\Delta}^{\uparrow}_f$ / `flex_down_mw` $\overline{\Delta}^{\downarrow}_f$, window
`flex_duration_h` $H_f$, ramp `ramp_rate_mw_min`, and `availability_pct` $a_f$.

**(a) Curtailable / adjustable load.** Served demand $\tilde P_{f,t}$:
$$
P^0_{f,t}-a_f\overline{\Delta}^{\downarrow}_f\le \tilde P_{f,t}\le P^0_{f,t}+a_f\overline{\Delta}^{\uparrow}_f,\qquad
|\tilde P_{f,t}-\tilde P_{f,t-1}|\le 60\,\rho_f\Delta t.
\tag{4.2a}
$$
$\tilde P_{f,t}$ replaces $P^0_{f,t}$ in the bus balance.

**(b) Energy-conserving shiftable load** (recover deferred energy within window):
$$
\sum_{t}\tilde P_{f,t}\,\Delta t=\sum_{t}P^0_{f,t}\,\Delta t\quad\text{(daily)},\qquad
\Big|\textstyle\sum_{\tau\le t}(\tilde P_{f,\tau}-P^0_{f,\tau})\Delta t\Big|\le E^{\text{flex}}_f.
\tag{4.2b}
$$

**(c) Penalty / incentive.** Discomfort cost $\Delta J^{\text{DR}}=\sum_t\Delta t\sum_f w_f\,|\tilde P_{f,t}-P^0_{f,t}|$
(linearised with an auxiliary $\delta_{f,t}\ge\pm(\tilde P_{f,t}-P^0_{f,t})$), weighted by
load priority `priority`. Curtailment/shed of *firm* load keeps the existing ENS penalty.

### 4.3 Promote must-take sources to dispatchable 🆕

Currently `PVSystem`, `PVArrayDC`, `StaticGenerator(DC)` are subtracted from net load
(non-decision). Optionally expose curtailable variables, mirroring renewables:
$$
0\le p^{\text{pv}}_{r,t}\le \bar P^{\text{pv}}_{r,t},\qquad
\Delta J=-\sum_t\Delta t\sum_r c^{\text{curt}}_r p^{\text{pv}}_{r,t},
\tag{4.3}
$$
with $\bar P^{\text{pv}}_{r,t}$ the irradiance/profile-scaled MPPT cap. This lets the
optimiser curtail distributed PV under reverse-power or voltage limits instead of forcing
must-take injection.

### 4.4 Mobile storage with transport scheduling 🆕 ✅ implemented (exogenous schedule)

`MobileStorage` adds location state to a battery: `e_rated_mwh` $E_b$, efficiencies,
`status`∈{Stationary, InTransit, Deployed}, `target_bus`, travel/stay windows.

**Implemented (opt-in, LP-sized).** The relocation *timing* is taken as given input rather
than co-optimised, which removes the per-bus assignment binaries.  Each unit $b$ has a signed
power $p^{\text{m}}_{b,t}$ (+=discharge) and an energy state $e^{\text{m}}_{b,t}$, with an
exogenous connection bus $\text{loc}_{b,t}$ derived from `departure_time`/`arrival_time`/
`status` (at `bus` before departure, disconnected while in transit, at `target_bus` after
arrival):
$$
p^{\text{m}}_{b,t}\in
\begin{cases}[P^{\min}_b,P^{\max}_b],&\text{loc}_{b,t}\ \text{connected}\\[2pt]
\{0\},&\text{in transit},\end{cases}
\qquad
e^{\text{m}}_{b,t}=e^{\text{m}}_{b,t-1}-\tfrac{\Delta t}{\eta_b E_b}\,p^{\text{m}}_{b,t},\quad
\underline{\text{soc}}_b\le e^{\text{m}}_{b,t}\le\overline{\text{soc}}_b.
\tag{4.4}
$$
$p^{\text{m}}_{b,t}$ enters the balance (C1b) at the *time-varying* bus $\text{loc}_{b,t}$,
so a unit can carry stored energy from a depot bus to a load bus.  A no-op when no mobile
storage is present, preserving the exact baseline.

**Co-optimised relocation (opt-in `mobile_storage_corelocate`).** When enabled the schedule
itself becomes a decision over two candidate buses (origin $O$, target $D$): per step the unit
is connected at $O$, connected at $D$, or in transit (binaries $z^O_{b,t},z^D_{b,t},w_{b,t}$
with $z^O+z^D+w=1$, anchored at $O$ at $t=0$). Power is injected only at the connected bus via
gated per-bus injections $q^O,q^D$ ($p^{\min}z\le q\le p^{\max}z$); leaving a bus forces a
transit step ($z^i_{b,t-1}-z^i_{b,t}\le w_{b,t}$); a per-transit travel drain
$\chi_b D_b/E_b$ depletes the battery while $w=1$; and a minimum-stay window
$\sum_{\tau=t}^{t+\underline H-1} z^i_{b,\tau}\ge \underline H(z^i_{b,t}-z^i_{b,t-1})$
(from `t_stay_min_hr`) prevents oscillation. This is the larger MILP form of (4.4); it is a
no-op when no mobile storage is present and falls back to the exogenous schedule when off.


### 4.5 Virtual power plant (aggregated DER) 🆕

`VirtualPowerPlant` exposes an aggregate envelope at PCC bus `pcc_bus`:
$P^{\min}_v,P^{\max}_v$ (`pmin_mw/pmax_mw`), ramps `ramp_up/down_max_mw_min`,
regulation `p_regulation_up/down_mw`, and aggregate storage `e_storage_sum_mwh`. As a
single equivalent controllable resource:
$$
P^{\min}_v\le p^{\text{vpp}}_{v,t}\le P^{\max}_v,\quad
|p^{\text{vpp}}_{v,t}-p^{\text{vpp}}_{v,t-1}|\le 60\,R_v\Delta t,
\tag{4.5a}
$$
$$
e^{\text{vpp}}_{v,t}=e^{\text{vpp}}_{v,t-1}-\Delta t\,p^{\text{vpp,sto}}_{v,t},\quad
0\le e^{\text{vpp}}_{v,t}\le E^{\text{agg}}_v,
\tag{4.5b}
$$
with reserve contribution $r^{\uparrow}_{v,t}\le P^{\text{reg}\uparrow}_v$ feeding (C9).
Objective adds an aggregate bid cost $\sum_t\Delta t\,c_v\,p^{\text{vpp}}_{v,t}$. (Full
internal-DER unbundling is out of scope for the aggregate envelope.)

### 4.6 Microgrid PCC exchange + islanding 🆕

`Microgrid` has PCC limits `p_import_max_mw`/`p_export_max_mw`,
`p_exchange_min/max_mw`, and `operating_mode`∈{GridConnected, Islanded}. Exchange at
PCC bus, with an islanding indicator $o_{\mu,t}\in\{0,1\}$ (1 = grid-connected):
$$
-\,o_{\mu,t}\,\overline P^{\text{imp}}_\mu\le p^{\text{mg}}_{\mu,t}\le o_{\mu,t}\,\overline P^{\text{exp}}_\mu,
\tag{4.6a}
$$
so an islanded microgrid ($o_{\mu,t}=0$) exchanges zero power and must self-balance its
internal generation/load. When `islanding_capability` is false, $o_{\mu,t}\equiv1$.
V2G charging stations attach here as bidirectional loads with the §4.2(a) headroom plus a
discharge cap from `Charger.p_dis_max_kw`.

### 4.7 Storage cycle / degradation cost 🆕

`Storage` exposes `max_cycles`, `replacement_cost`, `daily_cycle_limit`,
`charge_bid_price`, `discharge_bid_price`. Add a throughput (cycle-aging) cost using an
auxiliary $g_{s,t}\ge0$ for gross throughput $|p^{\text{ess}}_{s,t}|$:
$$
g_{s,t}\ge p^{\text{ess}}_{s,t},\quad g_{s,t}\ge -p^{\text{ess}}_{s,t},\qquad
\Delta J^{\text{deg}}=\sum_t\Delta t\sum_s \frac{C^{\text{rep}}_s}{2N^{\max}_s E_s}\,g_{s,t},
\tag{4.7a}
$$
plus an optional daily cycle cap $\sum_t g_{s,t}\Delta t\le 2 N^{\text{day}}_s E_s$. Bid
prices map to $\Delta J^{\text{bid}}=\sum_t\Delta t\sum_s(\pi^{\text{dis}}_s p^{\text{dis}}_{s,t}-\pi^{\text{ch}}_s p^{\text{ch}}_{s,t})$
when the single power variable is split $p^{\text{ess}}=p^{\text{dis}}-p^{\text{ch}}$.

### 4.8 Explicit DC network with branch thermal limits 🆕

Replace the flat-voltage assumption (§2.3-C2) with DC voltage variables
$V^{dc}_{n,t}$ (or $\nu_{n,t}=V^{dc}_{n,t}-1$ small-signal) so DC line flows become
constrained. For DC branch $m=(n,n')$ with conductance $G_m=1/r_m$:
$$
P^{dc}_{m,t}=G_m\big(V^{dc}_{n,t}-V^{dc}_{n',t}\big)\,S_{\text{base}},\qquad
\big|P^{dc}_{m,t}\big|\le F^{dc,\max}_m=\text{`rate\_a\_mva`},
\tag{4.8a}
$$
$$
\text{(nodal)}\quad \sum_{c\in n}p^{\text{vsc,dc}}_{c,t}+\sum_{d\in n}p^{\text{dc,ess}}_{d,t}+\sum_{k}\!(\cdots)-\sum_{m\ni n}P^{dc}_{m,t}=D^{dc}_{n,t}.
\tag{4.8b}
$$
**Converter losses** (currently lossless coupling): VSC quadratic loss
$P^{\text{loss}}_{c}=a_c+b_c|p^{\text{vsc}}_{c,t}|+r_c (p^{\text{vsc}}_{c,t})^2$ is
piecewise-linearised, and the AC/DC coupling becomes
$p^{\text{vsc,dc}}_{c,t}=p^{\text{vsc}}_{c,t}+P^{\text{loss}}_c$; DC–DC uses
$\eta_k$ on the through-power as today. One DC bus per island is the voltage reference
$V^{dc}_{\text{ref}}=$ `v_dc_set_pu`.

### 4.9 Energy router (multi-port converter) 🆕 ✅ implemented

`EnergyRouter` aggregates `ports[]` (AC/DC, each with `pmax_mw`, `eta`, control mode).
Per port $j$ the signed net injection is split into a non-negative inflow/outflow pair
$p^{\text{in}}_{v,j,t},\,p^{\text{out}}_{v,j,t}\in[0,P^{\max}_{j}]$ (bus→router / router→bus),
so the bus sees the net injection $p^{\text{out}}_{v,j,t}-p^{\text{in}}_{v,j,t}$ and the
internal node conserves power with port losses:
$$
\sum_{j}\eta_{j}\,p^{\text{in}}_{v,j,t}=\sum_{j}p^{\text{out}}_{v,j,t}.
\tag{4.9}
$$
This generalises the two-port VSC/DC–DC coupling to $N$ ports.  No anti-simultaneity
binary is needed: a round-trip on one port loses $(1-\eta_j)$ that costed generation must
make up, so simultaneous in/out is never optimal.  AC ports enter their bus balance; DC
ports enter the DC nodal balance when the DC network is modelled, otherwise they fold into
the slack/copper-plate residual.

**Implementation note.** In the canonical pipeline an `EnergyRouter` is *expanded* during
network projection into internal DC buses + VSC + DC/DC converters
(`expand_energy_routers`), so a fully projected case is already optimised through the proven
two-port converter path and `energy_routers` is empty.  The model above (`enable_energy_router`,
default off) covers cases that reach the UC with explicit router ports still present; it is a
no-op when none survive projection, preserving the exact baseline.


---

## 5. Out-of-UC components (handled in adjacent layers)

`Shunt`, `Transformer3W`, `RegulatorControl`, `AsynchronousMotor` are reactive/voltage
devices: they belong to the **per-step AC-OPF/PF** layer (§2.4), not the active-power UC.
`Switch`, `CircuitBreaker`, `DCCircuitBreaker` are **topology** decisions owned by the
network-reconfiguration / resilience modules (`docs/network_reconfiguration_models.md`).
If co-optimised topology is wanted inside the production simulation, the line-status binary
$z_{\ell,t}$ and big-M flow gating from that document can be imported — flagged as a larger
follow-on (significantly bigger MILP).

---

## 6. Extended production-cost objective (full 🆕)

$$
\min\ \underbrace{\sum_t\Delta t\sum_g(c_1^g p_{g,t}+c_0^g u_{g,t})+\sum_{t,g}s^{\uparrow}_{g,t}}_{\text{thermal (current)}}
+\underbrace{\Delta J^{\text{ext}}}_{4.1}
+\underbrace{\Delta J^{\text{DR}}}_{4.2}
+\underbrace{\Delta J^{\text{deg}}+\Delta J^{\text{bid}}}_{4.7}
-\underbrace{\sum_{t,r}\Delta t\,c^{\text{curt}}_r p^{\text{ren}}_{r,t}}_{\text{RES (current)}}
+\underbrace{\kappa\!\sum_{t}\!\big(\textstyle\sum_g\varepsilon_g p_{g,t}+\sum_x\varepsilon_x p^{\text{imp}}_{x,t}\big)\Delta t}_{\text{carbon (opt.)}}
+\underbrace{\pi^{\text{ens}}\!\sum_{t}\!\text{ENS}_t}_{\text{reliability}}.
\tag{6.1}
$$

Each braced group is an independently selectable **objective term** (GUI: see §7).

---

## 7. Mapping to the GUI controls (constraints / solvers / objectives)

| GUI control | Math switch | Status |
|---|---|---|
| 机组组合求解器 = Auto/Native/HiGHS/**SCIP** | MILP backend for (2.1)–(C9) | ✅ wired (`uc_solver`) |
| 网络约束(DC潮流) | (C1b)+(C8) instead of (C1a) | ✅ wired (`enable_network_constraints`) |
| DC网络耦合 | (C2) | ✅ wired (`enable_dc_network_constraints`) |
| DC支路限值 | §4.8 transport flows | ✅ wired (`enable_dc_branch_flows`) |
| 旋转备用(%) | (C9) with $\rho$ | ✅ wired (`reserve_fraction`) |
| 机组组合 / OPF 开关 | UC on/off, AC-OPF replay | ✅ wired (`skip_uc`,`run_opf`) |
| 外部电网交易 | §4.1 + $\Delta J^{\text{ext}}$ | ✅ wired (`enable_external_grid`) |
| 需求响应 / 可转移负荷 | §4.2 + $\Delta J^{\text{DR}}$ | ✅ wired (`enable_demand_response`,`dr_shiftable`) |
| 可调光伏 | §4.3 | ✅ wired (`enable_dispatchable_pv`) |
| 微网PCC交换 | §4.6 | ✅ wired (`enable_microgrid`) |
| 储能退化成本 | §4.7 | ✅ wired (`enable_storage_degradation`) |
| 虚拟电厂(VPP) | §4.5 | ✅ wired (`enable_vpp`) |
| 能量路由器 | §4.9 | ✅ wired (`enable_energy_router`) |
| 移动储能 | §4.4 | ✅ wired (`enable_mobile_storage`) |
| 协同选址 | §4.4 co-optimised relocation | ✅ wired (`mobile_storage_corelocate`) |
| 优化目标 = 成本/碳排放/弃电/网损/加权 | enable/weight each brace in (6.1) | ✅ wired (`objective`,`objective_weights`) |

The objective value, the requested solver, and the active constraint set are already
returned by `/api/session/run_ts_pf` and `/api/session/run_annual_sim`
(`objective_value`, `uc_solver_requested`, `constraints{}`) and shown in the results panel.

## Current profile and Canvas contract (2026-07-12)

`run_ts_pf` materializes the active time-series binding before solving. Imported
scenario profiles, explicit load-profile maps, and default assignments must be
resolved into each `pf_system_snapshots[t]`; a consumer must not replace a
missing snapshot with the static base system.

With `keep_system_snapshots=true`, `TimeSeriesPFResult` retains, per step:

- `pf_results[t]` in authored bus/branch space;
- `opf_results[t]` when dynamic OPF is enabled;
- `pf_system_snapshots[t]` carrying the actual time-varying device state;
- `rich_results[t]` carrying identity-keyed component attribution.

The GUI summary response contains curves and aggregates. Complete Canvas data
is read lazily through `GET /api/session/tspf/frame?step=N`. The frame contains
AC/DC voltage, branch and circuit-breaker P/Q, Grid/generator/storage output,
VSC/DC-DC transfers, SOC, transformer terminals, and bus P/Q diagnostics. The
renderer updates only the result layer and never writes the snapshot values
back into the authored model.

Annual totals must report the number of converged/dropped steps. Non-converged
steps cannot be silently omitted and then presented as a complete annual total;
the result must either rescale under a declared policy or expose incompleteness.

---

## 8. Validation (implemented — `tests/test_uc_*.cpp`)

Every rich model ships with a dedicated Catch2 test on a small hand-checked system,
all passing with the SCIP backend (**75 assertions / 29 cases** total), plus a standing
baseline-regression check:

1. **Conservation / sanity:** with all rich-model flags off, the objective equals the
   current value — `ieee24_3area_acdc_expanded` 24 h ⇒ \$694{,}503.645, cross-checked
   Native = HiGHS = SCIP, and re-verified after each model was added.
2. **External grid:** cheap grid displaces local generation; gen-offline case is infeasible
   without the tie and feasible with it.
3. **Demand response:** shiftable load conserves horizon energy and shaves the peak; a high
   discomfort penalty suppresses any shed.
4. **Dispatchable PV / static gen:** over-supply is curtailed (claw-back) instead of forcing
   infeasibility; all four source types covered.
5. **Mobile storage:** a stationary battery shifts its full energy into the peak step; a unit
   in transit during the peak can only help the off-peak step (higher cost).
6. **VPP / Microgrid / Storage degradation / DC flows / Energy router:** each verified against
   the closed-form optimum on its small test system (envelope limits, islanding cost,
   throughput cap, line-rating transport, lossy multi-port conservation).
7. **Solver agreement:** the baseline and enabled sets match (within the 0.1 % MIP gap)
   across Native / HiGHS / SCIP.

> **Status:** §4 (§4.1–§4.9), the selectable objectives in §6, and the GUI controls in §7
> are all implemented, opt-in, and regression-clean. Mobile storage offers both an exogenous
> schedule and a co-optimised relocation MILP (`mobile_storage_corelocate`). Remaining future
> work is noted inline (e.g. §4.1 buy/sell binary, §5 co-optimised topology).

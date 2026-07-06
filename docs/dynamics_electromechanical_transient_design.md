> Documentation Sync (2026-07-06)
> Scope: design and mathematical-derivation document for the electromechanical (RMS/phasor) transient
> simulation module, written against the current repository, the local PowerSimulationsDynamics.jl
> checkout (`/Users/tianyangzhao/Codes/PowerSimulationsDynamics.jl`, "PSID"), and the local GridLAB-D
> checkout (`/Users/tianyangzhao/Codes/gridlab-d`, "GLD").
> Status: analysis and design reference; verify decisions against current source before execution.
> Source of truth: when text and implementation diverge, treat src/, include/, tests/, and CMake files
> as authoritative.

# Electromechanical Transient Simulation for Hybrid AC/DC Distribution Systems
## Mathematical Derivation, Architecture Design, and Assessment of the Current `dynamics` Module

This document is the consolidated theory + design reference for the `hacdcpf` transient
(`include/hacdcpf/dynamics`, `src/dynamics`) module. It has three goals:

1. **Derive the mathematics** of an electromechanical (RMS/phasor) transient simulator for hybrid
   AC/DC *distribution* systems: unbalanced three-phase networks, dynamic frequency, and rich
   inverter-based resources (IBR) — with full derivations, not just equation listings.
2. **Design the target architecture**, synthesizing the two reference open-source tools:
   PSID (simultaneous DAE, composed devices, balanced positive-sequence) and GridLAB-D
   (partitioned predictor–corrector, unbalanced three-phase current-injection network,
   QSTS↔deltamode co-scheduling).
3. **Assess the limits of the current module** as implemented today, consolidating and extending
   `docs/dynamics_psid_parity_upgrade_plan.md` and `docs/powersimulationsdynamics_gap_analysis.md`.

Related documents: `docs/transient_simulation.md` (original C++ design note),
`docs/dynamics_psid_parity_upgrade_plan.md` (PSID parity & efficiency plan; Phases 0–5),
`docs/powersimulationsdynamics_gap_analysis.md` (capability crosswalk),
`tools/psd_validation/psd_component_test_matrix.md` (live stop/go ledger).

---

## Table of contents

- **Part I — Reference architectures studied**
  - 1. PowerSimulationsDynamics.jl: the simultaneous-DAE archetype
  - 2. GridLAB-D: the unbalanced partitioned archetype
  - 3. Synthesis: what this module should take from each
- **Part II — Mathematical foundations**
  - 4. Reference frames, per-unit, and the phasor (RMS) approximation
  - 5. Network models: balanced RI, unbalanced three-phase, dynamic branches, DC network
  - 6. The unified DAE: mass-matrix and residual forms
  - 7. Frequency in a phasor simulator
- **Part III — Device model derivations**
  - 8. Synchronous machine family (and the unbalanced-network interface)
  - 9. Shaft models
  - 10. Excitation, governor, and PSS block library
  - 11. Inverter-based resources (GFL, GFM, industrial renewable models, IEEE 1547)
  - 12. Hybrid AC/DC devices: VSC, DC network, DC/DC, storage, PV
  - 13. Loads and motors
- **Part IV — Numerical methods**
  - 14. Initialization (consistent DAE equilibrium)
  - 15. Time integration
  - 16. Jacobians and sparse linear algebra
  - 17. Events and discontinuities
  - 18. Small-signal analysis
- **Part V — Software design**
  - 19. Target architecture and state layout
  - 20. C++ interfaces
  - 21. Validation strategy
- **Part VI — Assessment of the current `dynamics` module**
  - 22. What exists today
  - 23. Limits, by category
  - 24. Recommended sequencing

---

# Part I — Reference architectures studied

## 1. PowerSimulationsDynamics.jl: the simultaneous-DAE archetype

PSID (Julia; models follow Milano, *Power System Modelling and Scripting*, and Lara et al.,
arXiv:2301.10043) is a **balanced positive-sequence RMS simulator** whose defining choice is the
*simultaneous* solution of all differential and algebraic equations as one system.

### 1.1 One system function, two solvable forms

PSID builds a single monolithic evaluation kernel and exposes it in two mathematically equivalent
forms (`src/base/simulation_model.jl`, `src/models/system.jl`):

- **MassMatrixModel** — `M·ẋ = f(x)`, integrated with mass-matrix-aware stiff ODE methods
  (Rodas4/5 Rosenbrock, TRBDF2).
- **ResidualModel** — `r = M·ẋ − f(x) = 0`, integrated with implicit DAE solvers (Sundials IDA),
  with `differential_vars` marking which unknowns carry a derivative.

The two kernels (`system_residual!`, `system_mass_matrix!`) are line-for-line parallel; every device
model is written once and serves both. This duality is what makes PSID's validation contracts
well-defined (each test is stated against one or both forms), and it is the formulation this module
should adopt (see §6, §19).

### 1.2 State layout and network model

Global unknown vector (`src/base/simulation_inputs.jl`):

```
x = [ V_r(1:n_bus) ; V_i(1:n_bus) ; branch states ; device states ]
```

Bus voltages are **algebraic** unknowns (mass-matrix rows = 0). The network is a current-injection
balance evaluated matrix-free over a **rectangular real Ybus** (`src/utils/psy_utils.jl:44`):

```
Y_rect = [  Re(Y)  Im(Y) ]        [I_r]         [V_r]
         [ -Im(Y)  Re(Y) ] ,      [I_i] = Y_rect·[V_i]

0 = i_inj(x, V) − Y_rect · V      (algebraic network rows, src/models/network_model.jl)
```

There is **no nested network solve**: devices `+=` their injection currents into the balance and the
outer implicit integrator's Newton iteration resolves the network and the device states together.

### 1.3 Composed devices and inner variables

- `DynamicGenerator = Machine + Shaft + AVR + TurbineGov + PSS` (evaluation order:
  update inner vars → TG → PSS → AVR → machine → shaft; `src/models/device.jl`).
- `DynamicInverter = Converter + OuterControl + InnerControl + DCSource + FreqEstimator + Filter`
  (evaluation order: DC side → PLL → outer → inner → converter → filter).

Blocks communicate through a fixed **inner-variables buffer**, not through extra states
(`src/base/definitions.jl`): 9 slots for generators (`τe, τm, Vf, V_pss, VR, VI, ψd, ψq, Xad·Ifd`),
25 for inverters (`md, mq, Vdc, V/I at filter and converter nodes, ω_pll, θ_pll, ω_oc, θ_oc, …`).
`Ports` declare which states/inner-vars each block consumes; wrappers freeze index ranges and hold
mutable `Ref` set-points (`V_ref, ω_ref, P_ref, Q_ref`) so perturbations mutate references without
rebuilding. The library is therefore *combinatorial*: ~12 machines × ~12 AVRs × ~11 governors ×
~4 PSS, and GFL/GFM/VSM/dVOC/REGC-REEC-REPC inverters from the same six sockets.

### 1.4 Numerics in brief

- Jacobian by ForwardDiff of the whole system function; sparsity detected once, `jac_prototype`
  passed to the solver; DAE-shifted form `J − γM` assembled in place (`src/base/jacobian.jl`).
- Initialization: trust-region power flow → per-device back-solve of internal states/set-points →
  full-system NLsolve refinement with strict/relaxed tolerances (`src/base/simulation_initialization.jl`).
- Small-signal: partition AD Jacobian into `[g_y g_x; f_y f_x]`, reduce
  `A = M_dd⁻¹(f_x − f_y g_y⁻¹ g_x)`, eigenvalues + participation factors (`src/base/small_signal.jl`).
- Perturbations as DiscreteCallbacks editing Ybus (NetworkSwitch/BranchTrip), device status, or
  set-point `Ref`s (ControlReferenceChange), with times in `tstops`.
- Frequency: one global `ω_sys` (ConstantFrequency ≡ 1.0, or ReferenceBus = the ω-state of the
  device at the slack bus); every device keeps its own speed state (§7).

### 1.5 PSID's own limits (relevant to this module)

Balanced positive-sequence only; no islanding (isolating trip diverges); constant delays only
(DEGOV) and only in mass-matrix form; small-signal validation WIP; needs a frequency-controlling
device at the reference bus. These are exactly the axes on which a *distribution*-focused tool must
go further — unbalance, islanding/frequency re-anchoring, and protection.

## 2. GridLAB-D: the unbalanced partitioned archetype

GLD is an event-driven QSTS distribution simulator with a sub-second dynamic mode ("deltamode").
Its defining choices are the **unbalanced three-phase Newton current-injection network** and the
**fixed-step predictor–corrector device integration with the network re-solved inside every pass**.

### 2.1 QSTS ↔ deltamode co-scheduling

The core (`gldcore/deltamode.cpp`, `gldcore/exec.cpp:2186-2610`) alternates event-driven simulation
with deltamode windows. Modules vote each event pass on when they need deltamode
(`delta_modedesired`); inside a window, per-timestep module `interupdate` callbacks return
`SM_DELTA_ITER` (re-iterate this instant), `SM_DELTA` (advance), or `SM_EVENT` (ready to exit).
The timestep is the minimum over module `preupdate` requests, fixed for the window. Devices
themselves decide when dynamics are over (e.g. `|Δω|`, `|ΔV|` convergence criteria) — the tool
returns to QSTS only when *all* devices vote `SM_EVENT`. This two-regime design is the right
long-horizon pattern for distribution studies (QSTS between disturbances, dynamics across them).

### 2.2 Unbalanced network: Newton current injection with Norton devices

`solver_nr` (`powerflow/solver_nr.cpp/.h`) solves per-phase current-mismatch equations

```
ΔI_r = (P·V_r + Q·V_i)/|V|² + I_hist,r − (Y·V)_r
ΔI_i = (P·V_i − Q·V_r)/|V|² + I_hist,i − (Y·V)_i
```

over buses carrying an explicit phase mask (A/B/C/any subset/triplex/split-phase). Dynamic sources
enter as **Norton equivalents**: a *fixed* 3×3 shunt admittance folded into the bus diagonal
(`bus.full_Y`) plus an updateable current (`bus.DynCurrent`, refreshed through a per-device callback
`ExtraCurrentInjFunc` that may demand further network iterations until injections stop moving).
Three solver modes: `PF_NORMAL` (static), `PF_DYNINIT` (initialization back-solve with the swing
still slack), `PF_DYNCALC` (dynamics run; swing demoted to PQ once sources own frequency —
the hook that makes **islanded operation** representable).

### 2.3 Machines in sequence coordinates on an unbalanced network

`diesel_dg` (`generators/diesel_dg.cpp`) implements a 6th-order GENROU-like machine **in the
positive-sequence** and interfaces it to the unbalanced network by sequence-to-phase transformation
of its Norton admittance:

```
Y_abc = T · diag(Y0, Y1, Y2) · T⁻¹ ,  Y1 = 1/(Ra + jXd″),  Y2 = 1/(jX2),  Y0 = 1/(jX0)
```

so negative/zero-sequence load currents flow through the machine's X2/X0 paths, and the
negative-sequence current produces a braking torque `½·Rr·|I₂|²` added to the electrical torque
(§8.8). This is the canonical positive-sequence-machine ↔ phase-domain-network interface this
module needs.

### 2.4 Inverters: GFM droop, GFL PLL + current control, IEEE 1547

`inverter_dyn` implements grid-forming (P–f / Q–V droop with PI voltage loop and Pmax/Pmin limiter
PIs, CERTS-style), grid-following (per-phase or positive-sequence SRF-PLL, dq current PI with
decoupling feed-forward, optional pure current-source mode), volt-var / frequency-watt with
deadbands and ramp limits, a full IEEE 1547 (2003/2014/2018) over/under-voltage and frequency
ride-through state machine with violation-time accumulators and reconnect timers, and DC-side
coupling to battery/PV objects through a registered-device interface (`V_DC` pushed, `I_DC`
returned, or an explicit DC-bus capacitor ODE). All modes present the same Norton interface
(`e_source/(R_f + jX_f)`).

### 2.5 Numerics: fixed-step Heun with network in the loop

Devices integrate with a two-pass predictor–corrector: forward-Euler predictor, trapezoidal (Heun)
corrector, with the **network re-solved between the passes** (each pass is an `SM_DELTA_ITER`
re-iteration of the powerflow module's interupdate). Frequency is *measured* per node (angle-difference
or per-phase SRF-PLL filters, `node.cpp:4717`) after each solve; a single scalar
`powerflow::current_frequency` is written by a designated "keeper" machine — there is no COI.

### 2.6 GLD's own limits (relevant to this module)

Fixed-order (Heun) fixed-step integration; device↔network consistency enforced only by bounded
re-iteration (10 outer loops) rather than a true simultaneous solve; monolithic device models
(each governor/exciter hand-woven into `diesel_dg`); no small-signal analysis; frequency handling
by designation rather than physics. Strong at network realism; weak at solver rigor and model
composability — the mirror image of PSID.

## 3. Synthesis: what this module should take from each

| Concern | Adopt from | What, concretely |
|---|---|---|
| Formulation | PSID | One state vector `[V; branch; device]`, mass-matrix + residual duality, no nested network solve |
| Device architecture | PSID | Composed blocks over an inner-variable bus; ports; Ref-based set-points |
| Model library breadth | PSID | GENROU/GENSAL families, AVR/TG/PSS library, GFL/GFM/VSM/dVOC, REGC/REEC/REPC, DER_A |
| Small-signal | PSID | Schur-reduced Jacobian, eigen + participation |
| Initialization | PSID | PF → per-device back-solve → full-system Newton refinement (equilibrium certificate) |
| Unbalanced network | GLD | Per-phase current-injection algebraic rows, phase masks, sequence-to-phase Norton devices |
| Machine↔unbalance | GLD | Positive-sequence machine + X2/X0 sequence shunts + negative-sequence braking torque |
| Islanding / references | GLD | Swing demotion (`PF_DYNCALC`), device-owned frequency, reconnection logic |
| Long-horizon studies | GLD | QSTS ↔ dynamic-window co-scheduling; devices vote on entering/leaving dynamics |
| Protection / DER standards | GLD | IEEE 1547 ride-through state machines, volt-var/freq-watt with deadbands and ramps |
| Faults | GLD | Unbalanced fault stamps (SLG/LL/DLG/3φ, open conductor) as admittance edits |
| Hybrid AC/DC | native | Keep this repo's DC network/VSC/DC-DC scope — neither reference tool has it |

The unique position of this module is the **intersection**: PSID's DAE rigor and composability,
GLD's unbalanced network and DER realism, plus native hybrid AC/DC — no open tool offers all three.

---

# Part II — Mathematical foundations

## 4. Reference frames, per-unit, and the phasor (RMS) approximation

### 4.1 From instantaneous quantities to the dynamic phasor

Let a phase quantity be `u_a(t) = √2·U(t)·cos(ω₀t + φ(t))` with slowly varying envelope `U, φ`.
Define the **dynamic phasor** in the synchronously rotating frame at constant base speed
`Ω_b = 2πf₀`:

```
ū(t) = U(t)·e^{jφ(t)}   so that   u_a(t) = √2·Re[ ū(t)·e^{jΩ_b t} ].
```

For an inductor `v = L·di/dt` with `i(t) = √2·Re[ī(t)e^{jΩ_b t}]`:

```
v̄ = L·dī/dt + jΩ_b L·ī                                             (4.1)
```

The **RMS/phasor (electromechanical) approximation** drops the envelope derivative `L·dī/dt`
whenever the envelope time constant is far slower than `1/Ω_b`, leaving the familiar algebraic
`v̄ = jX·ī`. Three regimes follow:

1. **Algebraic network** (classical transient stability): drop `dī/dt` on all branches —
   the network is a complex admittance equation `Ȳ·V̄ = Ī` at every instant.
2. **Dynamic branches / dynamic phasor** (PSID `dynline_model.jl`): keep (4.1) on selected
   branches. In per-unit with `l = X` this gives
   `(l/Ω_b)·dī/dt = v̄_from − v̄_to − (r + jl)·ī`, i.e. two extra differential states per branch.
   This matters when branch L/R time constants approach controller bandwidths (converter-dominated
   feeders, low X/R distribution lines) and is required for PSID Tests 10/11/25/27.
3. **EMT**: keep instantaneous waveforms. Out of scope here, but the formulation below is written
   so dynamic-phasor branches can be enabled selectively — the practical middle ground.

The validity boundary of the RMS approximation for IBR-rich feeders: inner current-control loops
(hundreds of Hz bandwidth) interact with network L/R poles; keeping algebraic networks with such
controllers can *falsely stabilize or destabilize* the response. The mass-matrix formulation (§6)
makes per-branch/per-filter promotion from algebraic to differential a data choice, not a code
change — this is the single most important formulation lesson from PSID.

### 4.2 Frames and rotations

Three frames coexist, and being explicit about them removes an entire class of bugs (the Test-03
δ-drift in the parity ledger is an angle-convention bug of this kind):

- **Network frame (RI)**: common synchronous frame rotating at `Ω_b`, unknowns `V_r, V_i` per bus
  (per phase, in the unbalanced case).
- **Machine rotor frame (dq, Kundur/Sauer-Pai convention)**: q-axis leads, rotor angle δ measured
  to the q-axis. PSID (`src/models/ref_transformations.jl`):

```
[v_d]   [ sin δ  −cos δ] [v_r]                [i_r]   [ sin δ   cos δ] [i_d]
[v_q] = [ cos δ   sin δ ]·[v_i] = ri_dq(δ)·v,  [i_i] = [−cos δ   sin δ ]·[i_q] = dq_ri(δ)·i
```

  equivalently `v_d + jv_q = (v_r + jv_i)·e^{−j(δ−π/2)}`.
- **Converter control frame (PLL convention)**: `v_d + jv_q = (v_r + jv_i)·e^{−jθ}` so a locked PLL
  gives `v_d = |V|, v_q = 0`. PSID implements this as `ri_dq(θ + π/2)`. Machine and converter
  conventions differ by 90°; both must be documented per device and tested at initialization
  (a stationary no-event run is the regression test).

For the **unbalanced** network, phase quantities relate to sequence quantities through the
symmetrical-component transform with `a = e^{j2π/3}`:

```
      [1  1   1 ]                    [x_0]    1 [1  1   1 ] [x_a]
T =   [1  a²  a ] ,  x_abc = T·x_012, [x_1] = ─ [1  a   a²]·[x_b]
      [1  a   a²]                    [x_2]   3 [1  a²  a ] [x_c]
```

(GLD `diesel_dg.cpp:2276-2338` uses exactly this pair.) Sequence-defined devices (machines) stamp
`Y_abc = T·diag(Y0,Y1,Y2)·T⁻¹` into the phase-domain matrix; phase-defined devices (unbalanced
loads, single-phase DER) stamp directly.

### 4.3 Per-unit

System per-unit on a single `S_base`; each device holds its own machine base `S_dev` and converts
at the network port: `Ī_network = (S_dev/S_base)·Ī_device`. Reactance and inductance coincide
numerically in per-unit (`x = l`), which is why filter/branch ODEs are written with `l/Ω_b` as the
mass coefficient. Frequency is per-unit on `Ω_b` (ω = 1.0 nominal). DC quantities use a DC voltage
base per DC island and the same `S_base`, so AC/DC converter power balance needs no base conversion.

## 5. Network models

### 5.1 Balanced positive-sequence network (PSID form)

Unknowns `V = [V_r; V_i] ∈ ℝ^{2n}`. Static branches stamp the π-model into the complex Ybus,
stored rectangular-real (§1.2). Algebraic rows:

```
0 = g(x, V) = i_inj(x, V) − Y_rect·V                                  (5.1)
```

Device injections **add** into `i_inj`; loads subtract (negative injection). Evaluation is
matrix-free (sparse mat-vec, no factorization inside the residual). Shunts from dynamic-branch
capacitances are added to Y at evaluation time.

### 5.2 Unbalanced three-phase network (target form)

Generalize each AC bus to a **phase set** `Φ(b) ⊆ {a, b, c}` (plus neutral/triplex variants later).
Unknowns per bus: `{V_r^p, V_i^p : p ∈ Φ(b)}`. The admittance matrix is built from 3×3 (or
|Φ|×|Φ|) phase blocks:

- **Lines**: phase-impedance matrix `Z_abc` from Carson's equations / geometry, Kron-reduced to the
  phase set; series block `Y_s = Z_abc⁻¹`; shunt `Y_sh/2` each end. Stamps are the usual
  `[Y_s, −Y_s; −Y_s, Y_s]` block pattern per phase pair.
- **Transformers**: connection-dependent nodal blocks (Yg-Yg, Δ-Yg, …) built from the leakage
  admittance and the incidence of winding connections; the Δ-Yg block introduces the 30° shift and
  zero-sequence blocking naturally through the block structure.
- **Devices**: stamp Norton blocks per §5.4.

The algebraic rows keep the *same shape* as (5.1) — only the sparsity blocks change from 1×1
complex (2×2 real) to 3×3 complex (6×6 real). This is the key design invariant: **the unbalanced
extension changes the layout module, not the formulation** (§19). A per-bus phase mask (GLD's
bit-packing generalized) drives index allocation, so balanced studies degenerate to 1 "phase" per
bus with zero overhead.

Two practical GLD lessons to preserve:

1. **Phase existence is data**: single-phase laterals, two-phase taps, open-conductor faults all
   reduce to masking rows/columns; never assume 3 phases per bus in the layout.
2. **Devices see their own phase set**: a single-phase PV inverter stamps a 1-phase Norton; the
   machine stamps a full 3×3 sequence-derived block (§8.8).

### 5.3 Dynamic branches (dynamic phasor rows)

Per selected branch, promote series current to states (PSID `dynline_model.jl`):

```
(l/Ω_b)·dī/dt = (v̄_from − v̄_to) − (r + jl)·ī        →  real form:
(l/Ω_b)·di_r/dt = (v_from,r − v_to,r) − r·i_r + l·i_i
(l/Ω_b)·di_i/dt = (v_from,i − v_to,i) − r·i_i − l·i_r                (5.2)
```

Line-charging capacitance at a terminal promotes that bus's voltage from algebraic to differential:
`(c/Ω_b)·dv̄/dt = ī_cap − jc·v̄` — implemented purely as mass-matrix entries flipping from 0 to
`c/Ω_b` on the bus-voltage rows (PSID `_adjust_states!`). The same mechanism serves the unbalanced
case with per-phase `l`, `c` blocks (mutual coupling makes the mass block 3×3 rather than diagonal —
supported by keeping the mass matrix block-diagonal, not strictly diagonal, in the design).

### 5.4 Device–network interface: Norton stamping

Every shunt-connected device presents:

```
ī_inj(x, V̄) = ī_N(x) − Y_N·V̄                                        (5.4)
```

with a state-dependent source current `ī_N` and an optional *constant-between-events* Norton
admittance `Y_N` folded into the effective Ybus. Two disciplines, from the two tools:

- **PSID discipline**: put *everything* state-dependent in `ī_inj`, keep Ybus device-free.
  Cleanest for AD Jacobians; the Newton matrix still sees the coupling through `∂i_inj/∂V`.
- **GLD discipline**: fold the passive part (`Ra + jXd″` for machines, `R_f + jX_f` for inverters)
  into Ybus as `Y_N`; the residual current source is better-conditioned and fixed-point loops
  converge faster.

In a *simultaneous* formulation both are algebraically identical; choose per device for
conditioning. Rule adopted here: **fold constant passive admittances into Ybus** (machine
subtransient + sequence shunts, converter filters, load constant-Z parts); keep everything
state-dependent in `ī_N`. This keeps `∂g/∂V = −Y_eff + ∂ī_N/∂V` sparse and mostly constant.

### 5.5 DC network

DC buses carry a single real unknown `V_dc`. Algebraic DC rows mirror (5.1):

```
0 = i_dc,inj(x, V_dc) − G_dc·V_dc                                     (5.5)
```

with `G_dc` the conductance matrix of DC branches. Where DC capacitance matters (converter DC
links, cable capacitance), promote:

```
C_dc·dV_dc/dt = i_dc,inj − G_dc·V_dc                                  (5.6)
```

— again purely a mass-matrix entry. DC-side devices (DC/DC converters, battery, PV, VSC DC ports)
stamp Norton pairs `(i_N, G_N)` exactly as on the AC side. The full unknown vector interleaves AC
and DC network rows in one algebraic block (§19.2); there is no separate "DC solve".

## 6. The unified DAE: mass-matrix and residual forms

### 6.1 Statement

Collecting device differential states `x`, and network/algebraic unknowns `y = [V_ac; V_dc; …]`,
with inputs `u` (set-points) and events (topology edits):

```
ẋ = f(x, y, u, t)                                                     (6.1a)
0 = g(x, y, u, t)                                                     (6.1b)
```

Stack `z = [y; x]` (network first — PSID's ordering, which keeps the algebraic block leading and
the Jacobian arrowhead structure friendly to fill-reducing orderings). The two solver-facing forms:

```
Mass-matrix:  M·ż = F(z, t),  M = blkdiag(0_{|y|}, M_x)               (6.2)
Residual:     R(ż, z, t) = M·ż − F(z, t) = 0                          (6.3)
```

`M_x` is block-diagonal: identity rows for normalized ODE states (machines, controllers — their
RHS carries the 1/T factors), `l/Ω_b`, `c/Ω_b`, `C_dc` entries for physical-storage states
(filters, dynamic branches, DC links), and 0 rows for *device-internal algebraic states*
(e.g. algebraic current-balance rows inside a detailed converter). One kernel `F` serves both
forms — write every device once.

### 6.2 Index and solvability

System (6.1) is **index-1** iff `g_y = ∂g/∂y` is nonsingular along the trajectory. For the network
rows, `g_y ≈ −Y_eff + ∂i_inj/∂V`; singularity arises exactly when:

- an **island loses its voltage anchor** (no GFM/machine/slack in the island): the island's block of
  `Y_eff` has a zero eigenvalue with no compensating device stiffness. PSID declares islanding
  unsupported; GLD handles it by re-anchoring (swing demotion / GFM takes over). The design must
  *detect* island formation at every topology event (connected-components pass over the live
  admittance graph) and either re-anchor (a GFM device exists in the island) or shed the island
  (declare its rows dead and mask them) — never integrate through a singular `g_y`.
- a **leaf constant-power bus at collapse** (`|V| → 0` makes `∂(S/V̄)*/∂V` blow up): guard with the
  standard current-limit or Z-fallback on constant-power loads below a voltage floor (GLD folds
  P/Q loads to impedance under `all_powerflow_delta` low-voltage conditions; adopt a smooth blend
  `P(V) = P₀·(V/V_min)²` below `V_min`).

Index-1 consistency of initial conditions is §14.

### 6.3 Discontinuities

Events (§17) change `g` (topology), `u` (references), or device internal modes (limiters). Between
events, `F` should be C¹ in `z` for the integrators to keep design order. Two consequences baked
into the model library: (a) limiter and deadband blocks use the *anti-windup conditional-derivative*
formulation (§10.2) which is piecewise-smooth with state-dependent switching handled by the
integrator's error control; (b) genuinely discontinuous actions (trips, mode changes) are events —
integrate to the event time, apply, re-initialize algebraic consistency, restart the step.

## 7. Frequency in a phasor simulator

Frequency is *not* a single physical unknown in a phasor simulator; being precise about its four
distinct roles avoids the confusion that plagues many tools:

1. **Base speed `Ω_b`** — constant, defines the rotating frame. Never changes.
2. **Device speeds `ω_k`** — states: machine rotor speeds, GFM virtual-rotor speeds
   (`ω_oc`), PLL frequency estimates (`ω_pll`). These are the *physical* frequencies.
3. **System/reference frequency `ω_sys`** — the frame-anchoring convention needed because a phasor
   network has one angle gauge freedom. PSID's two policies:
   - `ConstantFrequency`: `ω_sys ≡ 1`; all device angle equations integrate against the fixed
     frame: `δ̇ = Ω_b(ω − 1)`. Angles drift together during sustained frequency excursions —
     numerically fine, cosmetically annoying for long runs.
   - `ReferenceBus`: `ω_sys` = the speed state of the device at the reference bus:
     `δ̇_k = Ω_b(ω_k − ω_sys)`. Angles stay bounded relative to the reference machine; requires a
     frequency-owning device at the reference bus, and re-selection when it trips/islands.
   The target design generalizes to a **per-island reference registry**: each island has an anchor
   (a designated GFM/machine, or COI); topology events re-elect anchors. This subsumes PSID's two
   policies and GLD's "keeper of frequency" scalar.
4. **Measured frequency `f_meas`** — an *output* filter on bus-voltage angles, for relays, 1547
   logic, and plotting (GLD `calc_freq_dynamics`): per-phase
   `SIMPLE`: `ḟ_meas = ((Δθ/Δt + Ω_b)/2π − f_meas)/T_f`, or a per-phase SRF-PLL. Never feed
   measured frequency back into the frame — only into device *logic* (ride-through, UFLS) and
   frequency-dependent load models.

**Center of inertia** (per island `I`):
`ω_COI = Σ_{k∈I} H_k S_k ω_k / Σ_{k∈I} H_k S_k` — a derived output (and an optional anchor choice),
with IBR contributing through their virtual inertia constants where defined (VSM `Ta`), zero
otherwise. **Frequency-dependent network effects** (`Y(ω)` in branch reactances) are second-order
in distribution feeders and deliberately omitted from the algebraic network; device models that
need local frequency use their own PLL/measurement states, mirroring both reference tools.

---

# Part III — Device model derivations

*(Sections 8–13: each model is given as (i) derivation sketch, (ii) the exact state equations as
implemented in the reference tool, with source anchors, (iii) interface variables — what it reads
from and writes to the inner-variable bus and the network.)*

## 8. Synchronous machine family

### 8.1 From flux linkages to the standard reduced models

Start from the Park-transformed stator/rotor flux-linkage equations (generator convention,
per-unit, rotor frame):

```
(1/Ω_b)·ψ̇_d = R_s·i_d + ω·ψ_q + v_d
(1/Ω_b)·ψ̇_q = R_s·i_q − ω·ψ_d + v_q                                   (8.1)
rotor circuits: ψ̇_f, ψ̇_1d, ψ̇_1q, ψ̇_2q  driven by  v_f = V_f,  0 (dampers)
```

with the flux–current relations `ψ = L(θ_sat)·i`. The **reduced electromechanical models** arise by
(a) eliminating rotor currents in favor of EMFs behind reactances
(`e_q' ∝ ψ_f`, `e_d' ∝ ψ_1q`, `e''` ∝ damper fluxes), and (b) optionally applying the RMS
approximation `ψ̇_d = ψ̇_q = 0` to the *stator* rows, which converts (8.1) into the algebraic stator
equations

```
v_d = −R_s·i_d + x″_q·i_q + e″_d ,   v_q = −R_s·i_q − x″_d·i_d + e″_q   (8.2)
```

Models that *keep* stator flux derivatives (Marconato, Sauer-Pai — 6th order) retain `ψ_d, ψ_q` as
states with `Ω_b`-fast dynamics; models that drop them (GENROU/GENSAL/Anderson-Fouad "simple"
variants) couple to the network purely algebraically. Both belong in the library: the 6th-order
forms matter when converter controls interact with stator-flux time constants.

### 8.2 Classical machine (0 electrical states) — PSID `machine_models.jl:13`

`e_q'` constant behind `x_d'`:

```
i_d = (1/(R²+x_d'²))·(x_d'(e_q' − v_q) − R·v_d)
i_q = (1/(R²+x_d'²))·(x_d'·v_d + R(e_q' − v_q))
τ_e = (v_d + R·i_d)·i_d + (v_q + R·i_q)·i_q
```

### 8.3 One-d-one-q (2 states) — `machine_models.jl:64`

```
T'_d0·ė_q' = −e_q' − (x_d − x_d')·i_d + V_f
T'_q0·ė_d' = −e_d' + (x_q − x_q')·i_q
```

with the 2×2 algebraic stator solve for `i_d, i_q` (saliency `x_d' ≠ x_q'`).

### 8.4 Round rotor GENROU/GENROE (4 states) — `machine_models.jl:562`

States `e_q', e_d', ψ_kd, ψ_kq`; requires `x″_d = x″_q = x″`:

```
T'_d0·ė_q'  = V_f − X_ad·I_fd
T'_q0·ė_d'  = −X_aq·I_1q
T″_d0·ψ̇_kd = −ψ_kd + e_q' − (x_d' − x_l)·i_d
T″_q0·ψ̇_kq = −ψ_kq + e_d' + (x_q' − x_l)·i_q

ψ″_d = γ_d1·e_q' + γ_d2·(x_d' − x_l)·ψ_kd ,  ψ″_q = γ_q1·e_d' + (1−γ_q1)·ψ_kq
[i_d; i_q] = [−R  x″; −x″  R]⁻¹·[v_d − ψ″_q; −v_q + ψ″_d]
X_ad·I_fd = e_q' + (x_d − x_d')(γ_d1·i_d − γ_d2·ψ_kd + γ_d2·e_q') + Se(ψ″)·ψ″_d
X_aq·I_1q = e_d' + (x_q − x_q')(γ_q2·e_d' − γ_q2·ψ_kq − γ_q1·i_q) + Se(ψ″)·ψ″_q·γ_qd
τ_e = i_d(R·i_d + v_d) + i_q(R·i_q + v_q)
```

with `γ_d1 = (x″_d − x_l)/(x_d' − x_l)`, `γ_d2 = (x_d' − x″_d)/(x_d' − x_l)²` (q analogous), and
saturation on the subtransient flux magnitude `ψ″ = √(ψ″_d² + ψ″_q²)`:
GENROU quadratic `Se(ψ″) = B(ψ″ − A)²/ψ″`; GENROE exponential `Se = B·ψ″^A`
(PSID `saturation_models.jl:5-23`). These are the exact PSS/E GENROU/GENROE per the PSID docs.

### 8.5 Salient pole GENSAL/GENSAE (3 states) — `machine_models.jl:660`

States `e_q', ψ_kd, ψ″_q`:

```
T'_d0·ė_q'  = V_f − X_ad·I_fd
T″_d0·ψ̇_kd = −ψ_kd + e_q' − (x_d' − x_l)·i_d
T″_q0·ψ̇″_q = −ψ″_q − (x_q − x″_q)·i_q
X_ad·I_fd = e_q'(1 + Se(e_q')) + (x_d − x_d')(i_d + γ_d2(e_q' − ψ_kd − (x_d'−x_l)i_d))
```

(GENSAE: saturation on ψ″ instead of e_q', extra `−Se·γ_qd·ψ″_q` term.)

### 8.6 Marconato / Anderson-Fouad / Sauer-Pai (6 states)

Marconato (PSID `machine_models.jl:225`) keeps stator fluxes:

```
ψ̇_q = Ω_b(R·i_q − ω·ψ_d + v_q) ,  ψ̇_d = Ω_b(R·i_d + ω·ψ_q + v_d)
T'_d0·ė_q'  = −e_q' − (x_d − x_d' − γ_d)·i_d + (1 − T_AA/T'_d0)·V_f
T'_q0·ė_d'  = −e_d' + (x_q − x_q' − γ_q)·i_q
T″_d0·ė″_q = −e″_q + e_q' − (x_d' − x″_d + γ_d)·i_d + (T_AA/T'_d0)·V_f
T″_q0·ė″_d = −e″_d + e_d' + (x_q' − x″_q + γ_q)·i_q
i_d = (e″_q − ψ_d)/x″_d ,  i_q = (−e″_d − ψ_q)/x″_q ,  τ_e = ψ_d·i_q − ψ_q·i_d
```

`γ_d = T″_d0·x″_d·(x_d − x_d')/(T'_d0·x_d')`. Anderson-Fouad = Marconato with `γ_d = γ_q = T_AA = 0`.
Sauer-Pai (`machine_models.jl:133`) uses `ψ″_d, ψ″_q` damper states with the `γ_d1/γ_d2` blending
of `e'` and `ψ″` in the stator currents. "Simple" 4th-order variants drop the two stator-flux rows.

### 8.7 The GLD 6th-order machine — cross-check

GLD's `diesel_dg` (`apply_dynamics`, `diesel_dg.cpp:3623`) implements the same GENROU-class
physics in complex EMF notation (states δ, ω, `E' = E_d' + jE_q'`, ψ_1d, ψ_2q, V_fd):

```
dψ_1d/dt = (−ψ_1d + E_q' − (x_d'−x_l)·I_r)/T″_d0
dψ_2q/dt = (−ψ_2q − E_d' − (x_q'−x_l)·I_i)/T″_q0
dE_q'/dt = [V_fd − E_q' − (x_d−x_d')·(I_r − (x_d'−x″_d)·(ψ_1d + (x_d'−x_l)I_r − E_q')/(x_d'−x_l)²)]/T'_d0
dE_d'/dt = [−E_d' + (x_q−x_q')·(I_i − (x_q'−x″_q)·(ψ_2q + (x_q'−x_l)I_i + E_d')/(x_q'−x_l)²)]/T'_q0
```

— algebraically identical to §8.4 after the EMF↔flux change of variables. Useful as an independent
numerical cross-validation oracle for the C++ GENROU implementation.

### 8.8 Interfacing a sequence-domain machine to an unbalanced network

This is the derivation the current module lacks entirely, and GLD's most valuable lesson.

The reduced machine models above are **positive-sequence** models (Park transform assumes a
symmetric machine; the dq frame maps *positive-sequence* stator quantities to DC). On an unbalanced
network the machine terminal sees `V_abc → (V_0, V_1, V_2)`. The machine responds per sequence:

- **Positive sequence**: the dynamic model — Norton source `Ē_1/(R_a + jx″)` behind
  `Y_1 = 1/(R_a + jx″)` where `Ē_1` is built from the subtransient EMFs and rotated by `e^{jδ}`.
- **Negative sequence**: the rotor turns at `+ω` while the negative-sequence field rotates at `−ω`;
  in the rotor frame the negative-sequence quantities appear at `2ω` — far above the phasor band —
  so the machine presents the passive **negative-sequence impedance**
  `Z_2 ≈ R_2 + jx_2`, `x_2 ≈ (x″_d + x″_q)/2`: `Y_2 = 1/Z_2`.
- **Zero sequence**: `Y_0 = 1/(jx_0)` through the grounding path (∞ impedance if ungrounded —
  drop the Y_0 term).

Stamp `Y_abc = T·diag(Y_0, Y_1, Y_2)·T⁻¹` (GLD `diesel_dg.cpp:1159-1164`), and inject the dynamic
Norton current as *balanced positive sequence*: `I_abc = (Ī_1, a²Ī_1, aĪ_1)`.

**Negative-sequence braking torque.** The `2ω` rotor currents induced by `Ī_2` dissipate in rotor
resistance; the associated average torque opposes rotation. Power balance: the negative-sequence
airgap power is `P_2 = R_r·|I_2|²` per unit (with `R_r ≈ 2(Re(Z_2) − R_a)` — GLD
`diesel_dg.cpp:1167`), giving the torque correction implemented in GLD's `Te`:

```
τ_e ← τ_e(positive-sequence, §8.4) + ½·R_r·|Ī_2|²                     (8.8)
```

The ½ factor follows GLD's convention with `R_r = 2(Re Z_2 − R_a)`, i.e. the term equals
`(Re Z_2 − R_a)·|Ī_2|²`. `Ī_2` is computed from the machine terminal currents each evaluation:
`Ī_2 = (Ī_a + a²Ī_b + aĪ_c)/3`. Under balanced conditions the term vanishes and the model reduces
exactly to §8.4 — the balanced library is a strict special case.

### 8.9 Machine block interface (composed architecture)

Reads inner vars: `V_f` (from AVR), terminal `V_r, V_i` (from network via wrapper). Writes:
`τ_e` (to shaft), `ψ_d, ψ_q, X_ad·I_fd` (to AVRs that need field current, e.g. ESAC1A), network
injection `Ī`. States: per model above. This matches PSID's 9-slot generator inner-var bus and is
the contract the C++ `Machine` block must implement (§19).

## 9. Shaft models

**Single mass** (PSID `shaft_models.jl:9`):

```
δ̇ = Ω_b·(ω − ω_sys)
2H·ω̇ = τ_m − τ_e − D·(ω − 1)/ω                                        (9.1)
```

(PSID divides the damping term by ω — torque vs power convention; document and keep consistent.)

**Five-mass shaft** (HP, IP, LP, EX + rotor; 10 states): per mass `i`,

```
δ̇_i = Ω_b(ω_i − ω_sys)
2H_i·ω̇_i = τ_i − D_i(ω_i − 1) − D_{ij}(ω_i − ω_j) − D_{ik}(ω_i − ω_k)
            + K_{ij}(δ_j − δ_i) + K_{ik}(δ_k − δ_i)
```

with spring constants `K` and mutual dampings `D` along the shaft chain; `τ_i` is the turbine
torque fraction on that mass (governor output distributed), and the generator mass carries `−τ_e`.
Needed for torsional/SSR-adjacent studies when IBR controls excite shaft modes.

## 10. Excitation, governor, and PSS block library

### 10.1 Design rule: a reusable transfer-function block kit

PSID composes every controller from ~15 reusable blocks (`src/models/common_controls.jl`), each
returning `(y, ẋ)` and a mass-matrix variant: `low_pass`, `low_pass_nonwindup`, `high_pass`
(washout), `lead_lag` (1st–8th order), `pi_block(_nonwindup)`, `integrator_(non)windup`,
`ramp_tracking_filter`, deadband, and gate/valve maps. **Adopt this exactly** — one tested
anti-windup implementation instead of one per controller. Canonical realizations:

```
Low-pass  K/(1+sT):        T·ẋ = K·u − x,                    y = x
Lead-lag  (1+sT₁)/(1+sT₂): T₂·ẋ = u − x,                     y = x + (T₁/T₂)·(u − x)
Washout   sK/(1+sT):       T·ẋ = u − x,                      y = (K/T)·(u − x)
PI (non-windup):           ẋ = { k_i·e   if y_min<y<y_max or e drives y inward; else 0 },
                           y = clamp(k_p·e + x, y_min, y_max)
```

The non-windup conditional derivative (IEEE 421.5 Annex) is the piecewise-smooth form referenced
in §6.3.

### 10.2 AVR family (PSID `avr_models.jl`, `docs/src/component_models/avr.md`)

- **AVRFixed** `V_f = const`; **AVRSimple** `V̇_f = K_v(V_ref − V_h)`.
- **SEXS** (2 states): lead-lag + first-order exciter with limits;
  `V_in = V_ref + V_pss − V_h`, `T_b·V̇_r = (1 − T_a/T_b)V_in − V_r`,
  `T_e·V̇_f = V_LL − V_f`, `V_LL = V_r + (T_a/T_b)V_in`.
- **AVR Type I (DC1-like, 4 states)**: exciter with saturation `Se = A_e·exp(B_e|V_f|)`:
  `T_e·V̇_f = v_r1 − V_f(K_e + Se)`, regulator `T_a·v̇_r1 = K_a(V_ref − v_m − v_r2 − (K_f/T_f)V_f) − v_r1`,
  stabilizing feedback `T_f·v̇_r2 = −((K_f/T_f)V_f + v_r2)`, measurement `T_r·v̇_m = V_h − v_m`.
- **ESAC1A/EXAC1** (5 states): brushless AC exciter with rectifier regulation
  `V_FE = K_d·X_ad·I_fd + K_e·V_e + Se(V_e)·V_e`, `I_N = K_c·X_ad·I_fd/V_e`, rectifier curve
  `f(I_N)` per IEEE 421.5; this is why the machine must export `X_ad·I_fd` on the inner-var bus.
- **EXST1/ESST1A/ST6B/ST8C/SCRX**: static exciters with transient-gain reduction, PI/PD non-windup
  blocks, and field-current limiters — all expressible in the §10.1 kit.

### 10.3 Turbine-governor family (PSID `tg_models.jl`; GLD `diesel_dg.cpp:3716-3857`)

- **TGOV1** (2 states): droop `ref = (P_ref − (ω−1))/R`;
  `T₁·ẋ₁ = ref − x₁` (valve, clamped), `T₃·ẋ₂ = x₁(1 − T₂/T₃) − x₂`;
  `τ_m = x₂ + (T₂/T₃)x₁ − D_T(ω−1)`.
- **GAST** (3 states): two lags + temperature limit `min()` select.
- **HYGOV** (4 states): governor + gate + penstock water column `h = (q/G)²`,
  `τ_m = A_t·h·(q − q_NL) − D_turb·Δω·G`.
- **DEGOV/DEGOV1** (diesel): electric box lead-lag + actuator + **combustion dead time** `T_d` —
  a true transport delay. PSID: DDE with constant lag (mass-matrix form only);
  GLD: circular-buffer delay line sized `T_d/Δt`. For the C++ design adopt the **buffer** approach
  (works with any integrator; exact for fixed step; interpolated for adaptive step) — the DDE
  machinery is not worth importing.
- **GGOV1** (GLD `diesel_dg.h:56-94`): the full GE model with triple low-value-select
  (PID / acceleration limiter / temperature-load limiter), actuator, and engine transport lag —
  the industrial reference for gas units; port from GLD.
- **P_CONSTANT** (GLD): PI power tracking — useful for DER prime movers.

### 10.4 PSS family (PSID `pss_models.jl`)

`PSSSimple` (`V_s = K_ω(ω−ω_sys) + K_p(ωτ_e − P_ref)`), **IEEEST** (7 states: 2nd-order filters +
2 lead-lags + washout + clamp), **STAB1**, PSS2A/B/C (dual-input ω/P with ramp-tracking filter —
built from the §10.1 kit). Output `V_pss` adds into the AVR summing junction.

## 11. Inverter-based resources

### 11.1 The six-socket decomposition

Adopt PSID's decomposition as the *architecture*, with GLD's compact models re-expressed inside it:

```
DynamicInverter = Converter ⊕ OuterControl ⊕ InnerControl ⊕ DCSource ⊕ FreqEstimator ⊕ Filter
evaluation order: DC side → FreqEstimator → Outer → Inner → Converter → Filter → network
```

Inner-variable bus (25 slots, PSID `definitions.jl:61-87`): modulation `m_d, m_q`; `V_dc`;
filter-node voltages/currents; converter voltages/currents; `ω_pll, θ_pll`; `ω_oc, θ_oc, V_oc`;
current references. Each socket is independently swappable — that is what makes
GFL/GFM/VSM/dVOC/REGC variants *data*, not code.

### 11.2 Averaged converter and filter

**AverageConverter**: the switching bridge averaged over the carrier —
`v̄_cv = m·V_dc/2` with the modulation computed by inner control; in per-unit PSID reduces this to
`v_cv^{dq} = v^{ref,dq}` (ideal tracking), keeping `V_dc` influence through the DC source socket.
For hybrid AC/DC studies **do not** idealize: keep `v_cv = m·V_dc/V_dc,base` so DC-side dynamics
propagate to the AC side (§12).

**LCL filter** (6 states; PSID `filter_models.jl:35` — coefficients live in the mass matrix):

```
(l_f/Ω_b)·i̇_cv,r = v_cv,r − v_o,r − r_f·i_cv,r + ω_grid·l_f·i_cv,i     (converter inductor)
(c_f/Ω_b)·v̇_o,r  = i_cv,r − i_g,r + ω_grid·c_f·v_o,i                   (capacitor)
(l_g/Ω_b)·i̇_g,r  = v_o,r − v_grid,r − r_g·i_g,r + ω_grid·l_g·i_g,i     (grid inductor)
```

(+ imaginary rows with the `ω` cross-couplings negated). These are (4.1) applied to each element —
the filter *is* a dynamic-phasor sub-network. `RLFilter` is the algebraic reduction
`ī = (v̄_cv − v̄_grid)/(r_f + jl_f)` (GLD's fixed interface). Per-phase unbalanced filters follow
the same pattern with per-phase states (a GFL on phase *b* only carries that phase's states).

### 11.3 Frequency estimator (PLL)

**SRF-PLL derivation.** Project the measured voltage onto the estimated frame at angle `θ_pll`:
`v_q^{pll} = −v_r·sin θ + v_i·cos θ` (PLL convention: locked ⇔ `v_q = 0`). Feedback
`ω = ω₀ + PI(v_q)` drives `v_q → 0`. Small-signal: `v_q ≈ |V|·(θ_grid − θ_pll)`, so the closed loop
is a 2nd-order servo with `ω_n = √(k_i|V|Ω_b)`, `ζ = k_p√(|V|Ω_b/k_i)/2` — the designer's tuning
handles, and the reason PLLs weaken at low `|V|` (ride-through logic must freeze the PLL under
deep sags; GLD and REGC do exactly that).

- **KauraPLL** (4 states; PSID `frequency_estimator_models.jl:9`): low-passes both `v_d, v_q`
  before `atan`: `v̇_{d,pll} = ω_lp(v_d − v_{d,pll})` (idem q); `ε̇ = atan(v_{q,pll}/v_{d,pll})`;
  `θ̇_pll = Ω_b·(k_p·atan(·) + k_i·ε + 1 − ω_sys)`.
- **ReducedOrderPLL** (3 states): drops the d-channel filter.
- **FixedFrequency**: `ω_pll = 1` (GFM without PLL).
- **Per-phase PLLs** (GLD `inverter_dyn.cpp:3811`): one SRF-PLL per phase (or on the
  positive-sequence extraction) — required on unbalanced feeders; under unbalance a
  positive-sequence PLL needs a notch/DSC at `2ω` to reject the negative-sequence ripple
  (design note for the unbalanced GFL socket).

### 11.4 Outer controls (the "what to inject" layer)

- **GFL PQ (ActivePowerPI/ReactivePowerPI)**: PI on power errors produce current references
  `i_d^ref = PI_p(P_ref − p̃)`, `i_q^ref = PI_q(Q_ref − q̃)` in the PLL frame (`ω_oc = ω_pll`);
  measured `p = v_r i_r + v_i i_i`, `q = v_i i_r − v_r i_i`, low-passed.
- **GFM droop (ActivePowerDroop/ReactivePowerDroop)**:
  `θ̇_oc = Ω_b(ω_oc − ω_sys)`, `ω_oc = ω_ref + R_p(P_ref − p̃)`, `V_oc = V_ref + k_q(Q_ref − q̃)`,
  with `ṗ̃ = ω_z(p − p̃)` etc. The droop slope + measurement filter *is* a first-order virtual
  inertia: `2H_eq = 1/(R_p·ω_z)` — worth exposing as a derived diagnostic.
- **VSM (VirtualInertia)**: `ω̇_oc = (P_ref − p_e − k_d(ω_oc − ω_pll) − k_ω(ω_oc − ω_ref))/T_a`,
  `θ̇_oc = Ω_b(ω_oc − ω_sys)` — the swing equation (9.1) with `T_a = 2H`, damper `k_d` referenced
  to the PLL speed. Q-side as droop.
- **dVOC (ActiveVirtualOscillator)**: with `γ = ψ − π/2`,
  `ω_oc = ω_sys + (k₁/E²)(cos γ·ΔP + sin γ·ΔQ)`,
  `Ė = Ω_b[(k₁/E)(−sin γ·ΔP + cos γ·ΔQ) + k₂(V_ref² − E²)E]` — droop-like near equilibrium,
  globally synchronizing far from it.
- **REPC/REEC (ActiveRenewableControllerAB/Reactive…)**: the WECC plant/electrical controllers
  with `Freq_Flag/REF_Flag/PF_Flag/V_Flag/Q_Flag` branches — plant-level V/Q or PF control with
  deadbands and rate limits feeding electrical-control current commands. Implement flag combos as
  data-selected block graphs (PSID `docs/src/generic.md` lists supported combos).

### 11.5 Inner controls (the "how to inject" layer)

**GFM VoltageModeControl** (6 states; PSID `inner_control_models.jl:153`) — cascaded voltage/current
PI in the `θ_oc` frame with virtual impedance and active damping:

```
virtual impedance:  v_d^{vi} = V_oc − r_v i_d,g + ω_oc l_v i_q,g   (+q row)
voltage PI:         ξ̇_d = v_d^{vi} − v_d ;  i_d^{cv,ref} = k_pv(v_d^{vi} − v_d) + k_iv ξ_d
                                             − c_f ω_oc v_q + k_ffi i_d,g
current PI:         γ̇_d = i_d^{cv,ref} − i_d,cv ;
                    v_d^{ref} = k_pc(i_d^{cv,ref} − i_d,cv) + k_ic γ_d − ω_oc l_f i_q,cv
                               + k_ffv v_d − k_ad(v_d − φ_d)
active damping:     φ̇_d = ω_ad(v_d − φ_d)
```

The feed-forward/decoupling terms (`ω l i`, `ω c v`) cancel the cross-couplings introduced by the
rotating frame in §11.2 — that is their derivation, not a heuristic.

**GFL CurrentModeControl** (2 states): the current PI rows only, references from the outer PQ loop.

**GLD compact forms** (`inverter_dyn.cpp:4090`): identical current PI with `X_f` decoupling; the
GFM path replaces cascaded PI by direct `E∠θ` construction from droop outputs — re-express both as
socket combinations (`droop outer + algebraic inner + RL filter` ≈ GLD GFM; useful low-order
profiles for feeder-scale studies).

**Current limiting** — the defining IBR nonlinearity, three canonical strategies (PSID
`saturation_models.jl:93-141`; GLD `Imax` logic):
(a) **magnitude clamp** on `(i_d^ref, i_q^ref)` preserving angle;
(b) **priority clamp** — Q-priority under sags (voltage support, IEEE 2800) or P-priority;
(c) **virtual-impedance backoff** for GFM (raise `r_v, l_v` as `|i| → i_max` so the *voltage source
character is preserved* while current is bounded — hard saturation of a GFM turns it into a GFL
and destabilizes islands). Limiter state must be an explicit reported mode, and anti-windup on the
upstream PIs is mandatory (§10.1).

### 11.6 Industrial renewable converter (REGC_A) and DER_A

**REGC_A** (PSID `converter_models.jl:42`; 3 states): current-command lags
`T_g·İ_p = I_pcmd − I_p`, `T_g·İ_q = I_qcmd − I_q`, voltage filter `T_fltr·V̇ = V_t − V`;
low-voltage active-current management (LVACM) gain ramps `I_p` down under sags; high-voltage
reactive-current management (HVRCM) injects `I_q` correction; injection rotated to the network
frame. Combined REGC+REEC+REPC = the WECC generic PV/wind/BESS plant — the parity target for
utility-scale IBR. **DER_A** is the aggregate distribution-DER equivalent (PSID has it minus
tripping; GLD's 1547 machinery supplies the tripping logic — combine both).

### 11.7 IEEE 1547 functions (from GLD, `inverter_dyn.h:182-245`)

Volt-var and frequency-watt: piecewise-linear droops with deadbands, through low-pass
`T_qf/T_pf` filters and ramp-rate limits, feeding `Q_ref/P_ref` of any outer control.
Ride-through: per-category (2003/2014/2018) voltage/frequency band tables, each band with a
violation-time accumulator `ṫ_v = 1{V ∈ band}`, trip on `t_v > t_clear`, reconnect after
`t_reconnect` with ramped power restoration. These are *device modes + timers* — implement as a
protection block attached to the inverter, evaluated on measured (filtered) quantities (§7 role 4),
generating trip/reconnect events through the standard event queue (§17).

## 12. Hybrid AC/DC devices

*(This is native scope — neither reference tool covers it; models below are the design.)*

### 12.1 VSC as the AC/DC bridge

A VSC is a DynamicInverter (§11) whose DCSource socket is **replaced by a DC-network port**:

```
AC side:  v̄_cv = m̄·v_dc/2   (modulation from inner control; §11.2 non-idealized)
DC link:  C_dc·v̇_dc = i_dc,net − p_cv/v_dc                            (12.1)
          p_cv = Re(v̄_cv·ī*_cv)   (converter power, loss model σ(i) optional)
DC port:  stamps Norton (i_N = −p_cv/v_dc + C-term, G_N) into DC rows (§5.5)
```

Control modes map onto outer-control sockets: P/Q (GFL), V_ac (GFM), **V_dc control** (a PI on
`v_dc` producing `i_d^ref` — the DC slack), and **DC droop** `p_ref = p₀ − k_dc(v_dc − v_dc,ref)`
for multi-terminal sharing. The existing static `VSCConverter` role taxonomy (PQ/AC-PV/AC-GFM/
DC-GFM/DC-droop) projects one-to-one onto these socket combinations — preserve that projection.

### 12.2 DC/DC converter

Averaged model between DC buses `f → t`: duty/ratio state `d` with control PI,
power balance `p_t = η·p_f`; either algebraic (stiff feeder studies) or with an inductor state
`(l_dc)·i̇ = v_f − d·v_t − r·i` for interleaved converter dynamics. Stamps Norton pairs on both DC
buses.

### 12.3 Storage and PV on DC

Battery: OCV-R (or RC) equivalent `v_b = OCV(soc) − R_b·i_b`, `ṡoc = −i_b/(3600·C_Ah)`; SOC is a
*slow* state — keep it out of the fast equilibrium residual at initialization (§14.4). PV:
algebraic irradiance-dependent I–V curve (single-diode reduced), optionally MPPT lag
`T_mppt·v̇_ref = v_mpp − v_ref`. Both stamp DC Norton pairs; both may also appear behind a §11
inverter as its DCSource socket — same block, two mounting points.

## 13. Loads and motors

**ZIP + exponential** (PSID `load_models.jl:39`): aggregate per bus,

```
ī_load = [ (P_z + jQ_z)/V̄* ]  ⇒ stamped as constant Y  (fold into Ybus)
       + (P_i + jQ_i)·(V̄/|V|)/|V̄|*-form                 (constant current)
       + (P_p + jQ_p)/V̄*                                (constant power, with §6.2 low-V guard)
       + P₀(|V|/V₀)^α + jQ₀(|V|/V₀)^β  contributions    (exponential)
```

Frequency dependence `P = P₀(1 + k_pf·(f_meas − 1))` uses *measured* frequency (§7 role 4).
Unbalanced loads stamp per phase with delta/wye distinction (GLD's `S_dy` split — delta loads
convert to equivalent phase-to-phase injections; keep both wiring options in the data model).

**Induction machine** (single-cage, 3rd/5th order): states `ψ_dr, ψ_qr, ω_r` (+ stator fluxes for
the 5th-order form);

```
ψ̇_dr = −Ω_b·(R_r/X_rr)·(ψ_dr − X_m·i_ds… ) + Ω_b·s·ψ_qr , …
2H_m·ω̇_r = τ_e − τ_L(ω_r),  τ_L = τ₀·(A·ω_r² + B·ω_r + C)
```

— the dominant dynamic load in distribution feeders (fault-induced delayed voltage recovery);
PSID has both orders (`SingleCageInductionMachine`), port them into the composed-load family.
**ActiveConstantPowerLoad** (PSID): a rectifier-behind-controls CPL for electronics-rich feeders —
destabilizing negative incremental resistance captured dynamically.

---

# Part IV — Numerical methods

## 14. Initialization

### 14.1 The contract

A properly initialized simulation **must remain stationary until perturbed**:
`f(x₀, y₀, u₀) = 0 ∧ g(x₀, y₀, u₀) = 0`. The existing `dynamic_fast_dxdt_inf_norm` diagnostic is
the right observable; the target is to make it a hard gate (fail loudly above tolerance).

### 14.2 Three-stage procedure (PSID's, generalized)

1. **Power flow** on the hybrid AC/DC network (this repo's native solver — already
   unbalance-capable on the PF side) → `V₀` (all phases), `V_dc,0`, injections `S₀`.
2. **Per-device back-solve**: each device computes its internal states and *free set-points* from
   its terminal condition. Machine chain: `Ī = (S₀/V̄₀)*` → `δ₀` from
   `Ē = V̄₀ + (R_a + jx_q)Ī` (§8) → dq currents → EMF states → `V_f⁰, τ_m⁰` → AVR states back-solved
   from `V_f⁰` (this *determines* `V_ref`), governor states from `τ_m⁰` (determines `P_ref`).
   Inverter chain (PSID `initialization.md` order): filter states from terminal flow → PLL locked
   (`θ_pll = θ terminal`, `v_q,pll = 0`) → outer control (`θ_oc, V_oc/E₀`, `P_ref, Q_ref`) →
   DC source (`v_dc⁰`) → inner control (PI integrator states s.t. PI inputs are zero, modulation
   `m₀ = v_cv⁰/v_dc⁰`).
3. **Full-system refinement**: Newton on `F(z) = 0` (the §6 residual with slow states pinned,
   §14.4) from the stage-2 seed, strict tolerance `1e-9`, relaxed fallback `1e-6` with a warning —
   never silently accept drift.

### 14.3 Why stage 3 is not optional

Stage 2 is exact only device-by-device; shared quantities (bus voltages under device Norton
approximations, inner-var couplings, saturation) leave `O(tol_pf)` residuals that show up as
spurious 0.1–1 s transients and — worse — bias trace-parity comparisons (this is the standing
hypothesis for the Test-03 Marconato δ-drift in the ledger). The full-system solve collapses these
to solver tolerance.

### 14.4 Slow states and free parameters

States with hour-scale dynamics (SOC, thermal) are **pinned** during equilibrium (their rows
removed from the residual, values from the operating point); their derivatives are *allowed* to be
nonzero at t₀. The partition differential/algebraic/pinned is device-declared metadata, not solver
heuristics.

## 15. Time integration

### 15.1 Method selection

For the simultaneous DAE (6.2)/(6.3):

| Method | Form | Order | Use |
|---|---|---|---|
| Implicit trapezoidal (+ BE starts) | mass-matrix | 2 | workhorse fixed-step; α-damped variant to kill oscillations at events |
| TR-BDF2 | mass-matrix | 2 (L-stable) | events/stiff segments; no trapezoidal ringing |
| Rosenbrock (Rodas-class) | mass-matrix | 3–5 | small/medium systems, tight tolerances; needs exact Jacobian |
| BDF (IDA) | residual | 1–5 adaptive | large stiff systems, production adaptive path |
| Partitioned Heun (GLD-style) | partitioned | 2 | back-compat oracle; QSTS-embedded fast studies |

The mass-matrix Newton system at each implicit stage, e.g. trapezoidal:

```
Φ(z_{n+1}) = M(z_{n+1} − z_n) − (Δt/2)[F(z_{n+1}) + F(z_n)] = 0
∂Φ/∂z = M − (Δt/2)·J,   J = ∂F/∂z                                    (15.1)
```

— one sparse factorization per Newton iteration (reused across iterations/steps under modified
Newton with a staleness test on the residual convergence rate). BDF/IDA needs the shifted form
`J − γM` — the same assembly with a different scalar (PSID `jacobian.jl:73-86`).

### 15.2 Stiffness reality check

RMS phasor systems are stiff by construction: machine `T″ ~ 0.03 s` and `Ω_b`-scaled stator/filter
rows (`l_f/Ω_b ~ 10⁻⁴ s`) against governor/plant-controller `~5–60 s`. Explicit methods are limited
by the fastest retained mode (Δt ≲ 2/|λ_max| — sub-ms with LCL filters), which is why the
partitioned-explicit path cannot scale to converter-rich feeders. A-/L-stable implicit methods make
Δt an *accuracy* choice (1–10 ms typical) rather than a stability one.

### 15.3 The GLD alternative and when to use it

GLD's fixed-step Heun with network re-solve per pass is simple and robust for *moderately* stiff
device sets, and its two-regime QSTS↔dynamic scheduling is the right outer architecture for
long-horizon distribution studies. Design decision: keep a partitioned Heun engine as (a) the
verification oracle for the DAE path, (b) the QSTS-embedded engine (§19.6), while the DAE engine is
the claim-grade transient solver.

## 16. Jacobians and sparse linear algebra

`J = [g_y g_x; f_y f_x]` with: `g_y = −Y_eff + ∂i_N/∂V` (dominant, mostly constant);
`g_x = ∂i_N/∂x` (tall thin device columns); `f_y, f_x` block-sparse per device. Strategy, in order:

1. **Analytic device blocks** — each block (machine/AVR/filter/PLL/…) contributes its local
   `∂f/∂(x,V)`; the network block is `−Y_eff` (exactly constant between events). PSID gets this
   "for free" via ForwardDiff; C++ should hand-code blocks for library devices (they are small and
   the equations are frozen above) with
2. **finite-difference coloring fallback** for user devices (graph-colored columns: cost = number
   of structurally distinct columns, not n), and
3. **factorization reuse**: symbolic analysis once per topology; numeric refactor only when the
   Newton convergence degrades or Δt/γ changes (standard modified-Newton policy).

Linear solver: Eigen SparseLU baseline; KLU (circuit-matrix optimized, cheap refactorization for
fixed sparsity) as the production default — matching both the existing `SparseLinearSolver`
wrapper's optional SuiteSparse path and PSID's IDA+KLU pairing.

## 17. Events and discontinuities

Event taxonomy (union of the three tools' sets):

| Class | Examples | Mechanics |
|---|---|---|
| Topology | branch trip/close, breaker, network switch | edit Y_eff (all four rectangular quadrants), symbolic re-analysis, island scan (§6.2) |
| Fault | 3φ/SLG/LL/DLG shunt, open conductor | stamp fault admittance per phase (unbalanced faults are per-phase Y edits — GLD `link_fault_on` catalog); clear = inverse edit |
| Device | gen/converter trip, reconnect | status flag → zero rows + injection; re-init on reconnect |
| Set-point | ControlReferenceChange (P/Q/V/ω_ref), load scale | mutate device Ref |
| Protection-generated | 1547 trips, UFLS/UVLS, relays | device logic emits events into the same queue |
| State | PerturbState | direct state edit (testing) |

Discipline (PSID callbacks + standard practice): integrate *to* the event time (event times are
mandatory stop points), apply the edit, **re-solve algebraic consistency** (network Newton with
frozen differential states — a degenerate case of §14.3), restart integration (BDF restarts at
order 1; one-step methods restart trivially). Limiter-induced switching stays *inside* the
integrator as piecewise-smooth RHS (§6.3) unless a device declares a hard switching surface, in
which case optional event *location* (bisection on the switching function) tightens accuracy.

## 18. Small-signal analysis

At an equilibrium `z₀`, with `J` partitioned per §16 and `M_dd` the differential mass block:

```
Δẋ = A·Δx ,   A = M_dd⁻¹·(f_x − f_y·g_y⁻¹·g_x)                        (18.1)
```

(the Schur complement eliminating algebraic rows; PSID `small_signal.jl:71-106`). Outputs:
eigenvalues `λ = σ ± jω_d`, frequency `ω_d/2π`, damping `ζ = −σ/|λ|`, participation factors
`p_{ki} = |Φ_{ki}·Ψ_{ik}| / Σ_k|Φ_{ki}·Ψ_{ik}|` mapped back to named device states — the tool that
turns "the simulation rings at 14 Hz" into "the PLL of inverter X interacts with the filter of Y".
For IBR-rich feeders this is not a luxury: control-interaction screening (PLL vs weak grid, GFM
droop vs line X/R) is the primary design use case. Dense `eig` on the reduced `A` suffices at
distribution scale (10²–10³ differential states); shift-invert Arnoldi is the escape hatch beyond.
Validation: PSID Test 36 (vs ANDES) is the parity gate.

---

# Part V — Software design

## 19. Target architecture and state layout

### 19.1 Layered design

```
HybridPowerSystem (rich models)
   ↓ projection + DynamicModelProfile (standard/model_name/parameter_set provenance)
DynamicModelCatalog → DynamicModelBuilder
   ↓
DaeSystem                      ← the keystone object (Phase 1 of the parity plan)
  ├─ DaeLayout                 (bus/phase/DC/branch/device index map, phase masks)
  ├─ MassMatrix                (block-diagonal; 0 = algebraic, blocks for l/Ω_b etc.)
  ├─ ResidualEvaluator F(z,t)  (network rows ⊕ device rows; matrix-free over Y_eff)
  ├─ JacobianAssembler         (analytic blocks + coloring fallback; −Y_eff network block)
  ├─ EventQueue / IslandTracker
  └─ ReferenceRegistry         (per-island frequency anchor, §7)
   ↓
Integrators:  Trapezoidal/TR-BDF2/Rosenbrock (mass-matrix) · IDA adapter (residual)
              · partitioned Heun (oracle/QSTS mode)
   ↓
SmallSignal (Schur reduction, eigen, participation)  ·  DynamicResults / GUI / CSV
```

### 19.2 Unknown vector (unbalanced hybrid layout)

```
z = [ V_r/V_i per (bus, phase in Φ(b))   ← AC algebraic (or differential if C-promoted)
    ; V_dc per DC bus                    ← DC algebraic (or differential if C_dc)
    ; dynamic-branch currents            ← differential
    ; device states, per device          ← differential/algebraic per device declaration ]
```

`DaeLayout` owns every index; devices never compute global indices. Balanced systems get
`|Φ(b)| = 1` (positive sequence) — the balanced simulator is the unbalanced one with a trivial
mask, one code path.

### 19.3 Composed devices

Port PSID's structure literally:

```cpp
struct GenInnerVars  { double tau_e, tau_m, Vf, Vpss, VR, VI, psi_d, psi_q, XadIfd; };
struct InvInnerVars  { /* 25 slots, §11.1 */ };

class Block {                       // Machine, Shaft, AVR, TG, PSS, Converter, Filter, PLL, …
  virtual int  nStates() const;
  virtual void declareMass(MassRows&) const;              // per-state mass entries / algebraic
  virtual void residual(Ctx&, Span x, Span F) const;      // reads inner vars + terminal, writes F
  virtual void jacobian(Ctx&, JacBlocks&) const;          // analytic; optional
  virtual void initialize(InitCtx&) ;                     // §14.2 back-solve
};
class ComposedGenerator : DynamicDevice { Machine* m; Shaft* s; AVR* a; TG* t; PSS* p; …ordered eval… };
class ComposedInverter  : DynamicDevice { Converter*, Outer*, Inner*, DCSource*, FreqEst*, Filter*; };
```

Existing monolithic GFL/GFM/SynchronousMachine become *profiles* (fixed socket combinations) so
current tests pass unchanged on the composed runtime — the non-regression proof of the refactor
(parity plan Phase 3 exit criterion). Set-points are `Ref`-like handles mutable by events.
The catalog (`DynamicModelCatalog.cpp`) stays the single source of truth; every block ships with a
catalog entry + builder mapping + trace test.

### 19.4 The three-phase device rule

Every block declares its **phase signature**: `PosSeq` (balanced device on the positive-sequence
lane), `PerPhase(mask)` (per-phase states/injections), or `SeqCoupled` (positive-sequence dynamics +
Y₂/Y₀ stamps, §8.8). The layout allocates accordingly; a `PosSeq` device on an unbalanced network
gets an automatic sequence-extraction adapter (its terminal sees `V₁`; its injection distributes as
balanced positive sequence) — exactly GLD's machine treatment, made systematic.

### 19.5 Solver stack

`DynamicSolverOptions` keeps the existing enum surface, adding formulation choice
(`Partitioned | MassMatrixDae | ResidualDae`), per-branch dynamic-phasor flags, island policy
(`Shed | ReAnchor`), and modified-Newton/factorization-reuse knobs. Linear solvers: Eigen SparseLU
default, KLU when SuiteSparse is detected (existing CMake machinery), IDA as an optional backend
(existing "explicit optional future backend" policy — now with the residual object it needs).

### 19.6 QSTS co-scheduling (from GLD, later phase)

A `SimulationScheduler` alternates QSTS snapshots (existing PF engine + controller/tap/DER
set-point logic) with dynamic windows triggered by events or device votes; dynamic windows end when
all devices report settled (`|Δω|, |ΔV|` criteria — GLD's exit votes). This is the long-horizon
resilience/hosting-capacity study mode; it reuses the same DaeSystem with QSTS providing the
initial conditions of each window.

## 20. C++ interface sketch (delta from today)

Keep: `DynamicDevice` stamping interface, event records, results/CSV/GUI surfaces, catalog/builder
pattern, `SparseLinearSolver` wrapper, and the pieces of the parity plan that have **already
landed** (see §22): the `MassMatrixDae` simultaneous path with its `DaeLayout`
(`DynamicSolver.cpp:698`), `SmallSignal` Schur reduction, dynamic RL branches, `FiveMassShaft`,
and the network factorization cache. The remaining delta, in dependency order:

1. **Harden the DAE numerics** (the current path now has modified Newton with
   Jacobian/factorization reuse, an analytic effective-network/admittance Jacobian block with
   finite-difference corrections for voltage-dependent device current sources, a first
   per-device current-Jacobian block behind `DynamicDevice::addJacobian` for loads, dynamic RL
   lines, GFL/GFM/VSC current sources, CSVGN1, DER_A state-current entries, DC/DC, storage, and PV,
   and selectable backward-Euler or trapezoidal `M·ż = F` stages with an embedded BE/TR adaptive
   estimator): remaining work is analytic differential-device blocks (`f_x`, `f_y`) for machines
   and controllers, TR-BDF2/BDF stages, and a `ResidualModel`-shaped API for IDA later.
2. **Composition** (`Block`/inner-vars, §19.3): generalize `MachineControlLink` (τ_m/V_f
   derivative override) and `InverterInnerVariableBus` (telemetry-only today) into the full
   generator/inverter inner-var buses; re-express the monolithic devices as profiles.
3. **Library expansion** per the test-matrix priority list (blocked-controller rows first:
   ESAC1A/EXAC1-class AVRs need `X_ad·I_fd` on the bus from step 2; GAST/HYGOV/DEGOV governors;
   PSS2A/B/C; LCL filter states + limiter families; REGC/REEC/REPC flags; DER_A tripping).
4. **Unbalanced dynamics** (`Φ(b)` masks, per-phase device states, §19.4 sequence adapters,
   §8.8 machine interface, measured frequency, island tracking) — the network is already
   phase-domain; this step upgrades the *devices* and the *frequency/reference* machinery.
5. **QSTS co-scheduling + IEEE 1547 protection** (§19.6, §11.7).

## 21. Validation strategy

Three oracles, each for what it is authoritative on:

- **PSID** (local, `tools/psd_validation`): balanced DAE trace + small-signal parity — the
  39-row component matrix stays the stop/go ledger; Phase-1 formulation work converts the 12
  `compare-limited` rows into comparable rows before any new model lands.
- **GridLAB-D** (local checkout + existing bridge): unbalanced *network* algebra (existing snapshot
  gate) and, new, unbalanced *dynamic* cross-checks — `diesel_dg` vs C++ GENROU+sequence-interface
  on the same feeder (§8.7/§8.8), `inverter_dyn` GFM/GFL vs the composed profiles, deltamode traces
  vs the partitioned-Heun path.
- **Analytic**: single-device step responses against closed-form transfer functions (§10 blocks),
  PLL small-signal against §11.3 formulas, island frequency nadir against the swing-equation
  aggregate.

Per-model claim ledger (standard family, implemented/omitted blocks, tolerances, oracle evidence) —
as specified in the gap-analysis doc; the equilibrium certificate (§14.1) is a hard gate on every
claim-grade case.

---

# Part VI — Assessment of the current `dynamics` module

*(Based on a full source investigation dated 2026-07-05. Note: the live code is **ahead of** the
planning documents — `dynamics_psid_parity_upgrade_plan.md` describes Phases 1/2/5 as future work,
but a mass-matrix DAE path, small-signal analysis, dynamic branches, and the five-mass shaft are
already implemented. The gap-analysis header counts are stale versus the live ledgers. Treat this
Part as the current statement of record; `src/` remains authoritative.)*

## 22. What exists today

### 22.1 Architecture and scope

- **Pipeline**: `run_transient_simulation` → `DynamicModelBuilder::build` (canonical projection →
  power flow → `DynamicNetwork` assembly → PF-seeded voltages → device instantiation) →
  `DynamicSolver::solve` (`DynamicSolver.cpp:1186`, `DynamicModelBuilder.cpp:1221`).
- **Network**: genuine **three-phase phase-domain (abc)** AC network — node index
  `3·bus + phase`, full 3×3 complex branch blocks with mutual coupling
  (`DynamicSystem.hpp:30-33, 93-101`), built from explicit per-phase R/X/B matrices
  (`use_phase_matrix`, `DynamicModelBuilder.cpp:903`) or synthesized from sequence data via the
  symmetrical-component transform (`:890-898`); per-phase faults (`DynamicEvent.hpp:32`) and
  per-phase loads with phase masks (`ThreePhaseDynamicLoad`). Plus a real DC conductance network
  (`DynamicSystem.cpp:417-449`). This is *more* network capability than PSID has.
- **Devices** (all in `BasicDynamicDevices.{hpp,cpp}`, "BDD"): 11 synchronous-machine variants
  (Classical, OneDOneQ, SimpleAF, AndersonFouad, SimpleMarconato, Marconato, SauerPai, GENROU,
  GENROE, GENSAL, GENSAE — equations verified against §8 forms), `FiveMassShaft`, governors
  (TGOV1, TGTypeI/II, IEEEG1), exciters (SEXS, IEEET1, AVRSimple, AVRTypeI/II), PSS (PSS1A,
  IEEEST, STAB1), GFM inverter with Droop/VSM/VOC outer modes, GFL inverter with
  SRF/Kaura/ReducedOrder/FixedFrequency PLLs plus freq-watt/volt-var droops and current limit,
  `VSCConverterDynamic` (GFM+GFL composite), DER_A subset, CSVGN1 SVC, PeriodicVariableSource,
  DC/DC, battery (P+SOC), PV, protection relay, differential `DynamicRLLine`, ZIP/I/Z/P loads.
- **Controller coupling**: `MachineControlLink` — governor/exciter override the machine-owned
  `τ_m`/`V_f` state derivatives; PSS output read algebraically by the exciter
  (`MachineControlLink.hpp:16,50`; BDD:2044-2088).

### 22.2 Numerics

- **Two solver families** (`DynamicSolverOptions.hpp:9`): partitioned
  (Euler/Heun/RK4 explicit; backward-Euler/trapezoidal Newton with dense FD state Jacobian;
  Rosenbrock-Euler) and **simultaneous `MassMatrixDae`** — unknown `u = [x; ReV; ImV; V_dc]` with
  a `DaeLayout`, algebraic network rows `I_inj − Y_eff·V = 0`, backward-Euler/trapezoidal Newton
  on a sparse Jacobian with analytic network/current blocks plus FD remainder, Armijo line search,
  modified-Newton factorization reuse, and embedded BE/TR adaptive rejection. The §6 formulation
  exists in first- and second-order one-step form.
- **Network solve (partitioned mode)**: fixed-point Gauss loop, max 6 iterations, tol 1e-6, with a
  **factorization cache** keyed on matrix equality — the parity plan's Phase 0 is done
  (`DynamicSystem.cpp:716`, `DynamicSystem.hpp:118-135`).
- **Initialization**: PF seeding → per-device init → equilibrium **trim** loop
  (network solve ⇄ `trimToNetworkEquilibrium` ⇄ controller re-anchoring, to
  `‖ẋ‖_∞ ≤ dynamic_trim_tol`) → optional global consistent-init Newton
  (`DynamicSystem.cpp:472-621`) with per-state residual diagnostics. This matches §14's shape.
- **Events**: branch trip/close/impedance-scale, per-phase fault shunt with auto-clear, load
  scaling, device trips, storage steps; steps land exactly on event times; numerical-health guards
  for voltage collapse/blow-up (`DynamicSolver.cpp:144-270, 376, 927-931`).

### 22.3 Analysis and validation assets

- **Small-signal analysis**: full-system FD Jacobian, Schur elimination of network algebraic
  variables `A_red = f_x − f_y g_y⁻¹ g_x`, eigenvalues/damping/participation
  (`SmallSignal.cpp`) — §18 exists.
- **DAE diagnostics exporter** in PSD coordinates (residual, mass diagonal, full and reduced
  Jacobians, eigenvalues) — currently wired for the Test-03 SimpleMarconato stack
  (`DynamicDaeDiagnostics.hpp:21-58`).
- **PSID parity ledgers** (`tools/psd_validation/`): model crosswalk 47 rows —
  3 exact-or-close / 35 supported-subset / 7 missing / 2 profile-only; component test matrix
  41 rows — **25 full-gate passing** (Tests 01–08, 12, 13, 15–19, 23–26, 28, 30, 41, 44, 49, 51:
  machines incl. GENROU/GENROE/GENSAL/GENSAE with saturation, AVR/PSS combinations, VSM/droop/VOC
  GFM, GFL both PLLs, dynamic RL line, five-mass shaft, CSVGN1), 5 compare-limited,
  6 blocked-missing-controller, 3 blocked-missing-formulation, 2 metadata-only.
  Suite verdict: **"can pass all groups now: false"** — 11 rows still blocked.
- **Latest numeric verification (2026-07-06)**: focused DAE/Jacobian gate passed
  17,880 assertions across the MassMatrixDae regression, device current-Jacobian finite-difference
  stamp contract, and hybrid AC/DC analytic-device trace preservation; full transient suite passed
  94,442 assertions in 53 cases; internal PSD manifest passed 1,295 assertions; external PSD
  comparison against `/Users/tianyangzhao/Codes/PowerSimulationsDynamics.jl` passed 776,678
  assertions.

## 23. Limits, by category

### 23.1 Formulation and integration numerics

| Limit | Evidence | Consequence |
|---|---|---|
| `MassMatrixDae` has backward-Euler and trapezoidal stages, but no TR-BDF2/BDF or variable order | `DynamicSolver.cpp:1016-1165` | Claim-grade oscillatory studies can now use a 2nd-order simultaneous DAE path; production BDF/IDA remains future work |
| DAE sparse Jacobian has an analytic effective-network/admittance block, first per-device current blocks, FD remainder subtraction to avoid double-counting, and modified-Newton LU reuse | `DynamicSolver.cpp:849-1165`; `BasicDynamicDevices.cpp` current-source hooks | First-run cost and algebraic-current noise are reduced for covered devices, but FD column sweeps still limit scale until differential device blocks and coloring land |
| `addJacobian` now covers current-injection blocks for common loads, dynamic RL lines, GFL/GFM/VSC, CSVGN1, DER_A state-current entries, DC/DC, storage, and PV; machine/controller differential blocks remain FD-backed | `DynamicDevice.hpp`; `BasicDynamicDevices.cpp`; `tests/test_transient_dynamics.cpp` | The algebraic current-balance convention is now executable and numerically verified for the covered block; `f_x`/`f_y` still need analytic expansion |
| Adaptive stepping **off by default**; MassMatrixDae trapezoidal has an embedded BE/TR estimator, while partitioned implicit steppers still use **dense** FD state Jacobians | `DynamicSolver.cpp:444-666, 1210-1280` | Fixed-step remains the default for parity stability; adaptive DAE runs can now reject oversized steps without step-doubling |
| No `ResidualModel`-shaped API (IDA-ready object); PSD-coordinate diagnostics generic only for one stack | crosswalk rows `:80-81`; `psd_validation/README.md:222-243` | 3 `blocked-missing-formulation` rows; no production adaptive-BDF option |

### 23.2 Unbalance: a three-phase network driven by balanced devices

The single largest capability asymmetry. The network is honestly unbalanced (22.1), but **every
dynamic source reads `positive_sequence_voltage(...)` and injects balanced positive-sequence
current** (machines: BDD:2538, 2906; inverters: BDD:4747, 4818). Consequently:

- Machines present **no negative-/zero-sequence admittance** (no `Y₂/Y₀` stamps per §8.8) and feel
  **no negative-sequence braking torque** — an unbalanced fault looks to the rotor like its
  positive-sequence shadow only; SLG-fault machine response is quantitatively wrong.
- Inverter controls cannot see or respond to unbalance (no per-phase PLLs, no negative-sequence
  current control/limits), so unbalanced ride-through studies — a core use case for a
  distribution tool — are out of reach.
- Unbalance propagates through the passive network only; the validation notes correctly call this
  a "synthesized three-phase network solve" (`tools/psd_validation/README.md:182-184`).

The §19.4 phase-signature design (PosSeq / PerPhase / SeqCoupled) is the planned cure; §8.8 gives
the machine math; GLD supplies the cross-validation oracle.

### 23.3 Frequency

No center-of-inertia, no measured bus frequency, no per-island reference machinery: snapshot
`frequency_hz` is the **constant nominal** copied verbatim (`DynamicSolver.cpp:24`); frequency
exists only as per-device states (rotor ω, PLL, GFM internal). Missing versus §7: measured-frequency
filters (needed by relays/1547/frequency-dependent loads), COI reporting, per-island reference
re-anchoring — and therefore **no islanding support** (no island detection on topology events; an
island without a GFM/machine anchor makes `g_y` singular and the solve fails undiagnosed, §6.2).
Loads have no frequency dependence.

### 23.4 Device composition and the controller ceiling

The Phase 3 composition refactor now has executable generator and inverter inner-variable buses:
machine/controller blocks publish τ_m, V_f, V_pss, ψ_d/ψ_q-related telemetry, and GFL/GFM devices
publish converter, filter, PLL, outer-control, inner-control, and DC-link slots. This unlocked the
controller-family expansion and the first detailed GFL filter/limiter sockets. Remaining composition
work is narrower: full PSD REGC/REEC/REPC flag graphs, exact rectifier field-current feedback for
all AC/ST exciters, and promotion of these local block contracts into residual/mass-matrix trace
fixtures.

### 23.5 Model-library gaps (from the live ledgers)

- **AVR**: ESAC1A, EXAC1, EXST1, SCRX, ESST1A, ST6B, ST8C now have runtime/catalog variants and
  local derivative checks; full PSD residual/mass-matrix trace gates remain pending.
- **Governor**: GAST, HYGOV, DEGOV/DEGOV1, PIDGOV, WPIDHY, and TGSimple now have runtime/catalog
  variants and local derivative checks; PSD controller-by-controller trace fixtures remain pending.
- **PSS**: PSS2A/B/C now allocate the PSD-style state families and publish stabilizer outputs; the
  remote/electrical-power signal path still needs PSD fixture parity.
- **IBR detail**: GFL now has selectable LCL filter voltage states, current-limiter families, and
  GUI/catalog child-block parsing. Remaining gaps are REGC_A LVPL/ramps, REEC flag branches, REPCA
  plant controller, and exact PSD LCL mass-matrix coefficients.
- **DER_A**: voltage/frequency trip thresholds and delay timer now drive the multiplier state; PSD
  Test 42 trace gates are still pending.
- **Loads/motors**: simplified and full single-cage induction-machine runtime targets now exist for
  rich asynchronous motors. Exponential loads and active-CPL dynamics remain open.
- **Protection/standards**: relay is a bare voltage-window timer; no IEEE 1547 ride-through
  category tables, volt-var/freq-watt deadband curves are inverter-internal simplifications
  (§11.7 is the target); no UFLS/UVLS.
- **Hybrid AC/DC dynamics**: VSC is a GFM/GFL selector without the §12.1 modulation-level
  DC coupling (`v_cv = m·V_dc`); DC/DC is a single power-lag; no DC droop coordination dynamics.

### 23.6 Initialization and equilibrium smells

- A hard-coded **power-balance dead-band** `kMachinePowerBalanceTolPu = 1e-4` zeroes small swing
  residuals (BDD:23) — it holds equilibrium cosmetically but masks genuine sub-1e-4 drift, can
  bias PSID trace comparisons, and creates a non-smooth RHS kink inside the integrator (§6.3
  argues for removing it once §14.3's consistent init makes it unnecessary).
- Base-matrix assembly folds bus loads to **constant impedance at V₀** (`DynamicSystem.cpp:401`)
  unless a dynamic load device overrides — quietly changing load character between PF and dynamics
  for un-modeled buses.
- Exciter initialization prefers the PF terminal voltage because "network not yet solved here"
  (BDD:3696) — an ordering artifact the §14.2 staged procedure removes.
- Hard clamps (`H ≥ 0.01`, source reactance ≥ 1e-8, `singular_regularization_pu = 1e-8` on Y)
  are pragmatic but should be reported when active, not silent.

### 23.7 Validation status

Strong and honest where it exists: 25 full-gate PSID trace parities including an internal
residual/mass/Jacobian/eigenvalue gate for Test 03 (the δ-drift that blocked the old ledger is
resolved). Remaining: the 11 blocked rows (§23.4–§23.5 causes), diagnostics generalization beyond
the Test-03 stack, **no GridLAB-D *dynamic* cross-checks** (only static snapshots — §21 proposes
`diesel_dg`/`inverter_dyn` deltamode traces as the unbalanced-dynamics oracle), and no analytic
step-response gates for the block library.

### 23.8 Documentation drift

`dynamics_psid_parity_upgrade_plan.md` and `powersimulationsdynamics_gap_analysis.md` lag the
code (phases marked future are implemented; headline counts stale). Per the repo convention the
ledgers + source are authoritative; those two docs should be re-synced or superseded by this one.

## 24. Recommended sequencing

Updating the parity plan's phases to the current state:

1. **(was Phase 1/2 — largely done)** Harden what landed: §20 item 1 — modified-Newton
   Jacobian/factorization reuse, analytic effective-network/admittance block, trapezoidal DAE
   stage, embedded BE/TR adaptive stepping, and first per-device current-Jacobian blocks are now in
   place. Next: analytic differential blocks for machine/controller/inverter state equations,
   coloring fallback for remaining FD columns, TR-BDF2/BDF, and an IDA-ready residual API.
2. **Composition refactor** (§20 item 2 = old Phase 3) — still the keystone for every blocked
   controller row; behavioral-equivalence tests on existing profiles are the exit gate.
3. **Library expansion** (§20 item 3 = old Phase 4), ordered by the matrix: AVR family →
   governors → PSS2 → LCL/limiters → REGC/REEC/REPC flags → DER_A tripping → induction machine.
4. **Unbalanced dynamics + frequency/islanding** (§20 item 4, new): §8.8 machine interface with
   GLD cross-validation, per-phase inverter sockets, measured frequency, island tracker with
   per-island references. This is the step that makes the module *the* tool for its stated
   mission — no reference tool has this combination.
5. **QSTS co-scheduling and IEEE 1547 protection** (§20 item 5): long-horizon distribution studies.

Throughout: equilibrium certificate as a hard gate (§14.1); every new block ships with catalog
entry + builder mapping + oracle trace test (existing discipline); claim ledger per §21.

# Complete Mathematical Derivation for Extending Static Hybrid AC/DC Distribution Analysis to Three-Phase Transient Dynamics

Below is a complete mathematical framework for enhancing your existing hybrid AC/DC static distribution-system module into a **three-phase unbalanced phasor-domain transient simulation module**.

The derivation combines:

1. **Static hybrid AC/DC power-flow formulation** from your current module.
2. **Three-phase unbalanced network equations** similar to the GridLAB-D paper.
3. **Synchronous-machine electromechanical dynamics**.
4. **Grid-following inverter dynamics**.
5. **Grid-forming inverter dynamics**.
6. **VSC AC/DC converter dynamics**.
7. **DC/DC converter dynamics**.
8. **Battery, PV, and dynamic load models**.
9. **Unified differential-algebraic equation formulation**.
10. **Numerical solution procedure**.

The goal is to formulate a general model of the form:

$$
\dot{x} = f(x,y,u,t)
$$

$$
0 = g(x,y,u,t)
$$

where:

- $$x$$ is the vector of dynamic states,
- $$y$$ is the vector of algebraic network variables,
- $$u$$ is the vector of control references, disturbances, and events,
- $$t$$ is time.

## Transient Equilibrium and External AC Cross-Check

The pre-event point for transient stability must be a DAE equilibrium, not merely a numerically converged network solve. The static power flow provides the algebraic voltages and powers, then each dynamic device must initialize its internal states so that:

$$
f(x_0,y_0,u_0,t_0)=0,
\qquad
g(x_0,y_0,u_0,t_0)=0.
$$

If this consistency step is done correctly, the simulation should not need several artificial seconds to absorb initialization imbalance before the first scheduled contingency. Any control-frequency or PLL drift at the no-event start is therefore an initialization defect or a model-scope mismatch, not a required transient phenomenon.

GridLAB-D is used as an independent AC-scope check of $$g(x_0,y_0,u_0,t_0)=0$$ for the network part. The current harness exports the canonical AC network to a balanced three-phase GridLAB-D snapshot and compares bus voltages and branch powers when the exported model is in the shared PQ feeder scope. It validates rich-to-canonical projection, AC component parameters, transformer export, and parallel-line equivalents before dynamic devices are attached.

Cases outside that shared scope are still valuable diagnostics but not exact equivalence claims. MATPOWER-style PV buses are the main example: until GridLAB-D export includes equivalent voltage-regulating generator behavior, non-slack generators are represented as negative constant-power injections, so PV-heavy transmission cases such as case300 are run-only diagnostics. DC networks, converter controls, and transient state equations remain inside the `dynamics` module and are cross-checked in later stages through boundary injections and event replay.

The implemented GridLAB-D claim is now a report-level gate, not a prose
promise. `GridLABDComparisonReport::equivalence_passed` is true only when
GridLAB-D and HACDCPF both solve, at least one configured numerical comparison
is performed, every comparison is inside tolerance, and the preflight
classifier finds no unsupported or diagnostic-only features. Therefore the
allowable claim is:

> HACDCPF is numerically equivalent to GridLAB-D for this declared balanced AC
> algebraic snapshot within the configured tolerances.

It is not a claim of full GridLAB-D AC-distribution equivalence. Unbalanced
phase-domain elements, voltage regulators, protection/control sequences, DER
controllers, dynamic device equations, and hybrid AC/DC coupling remain separate
verification scopes until they have explicit import/export mappings and
point-by-point numerical checks.

For data exchange and component-level verification, the module also provides an
always-compiled OpenDSS/GridLAB-D text I/O layer. Its purpose is different from
the external GridLAB-D run harness:

$$
\text{HybridPowerSystem}
\longleftrightarrow
\text{OpenDSS DSS / GridLAB-D GLM}
\longleftrightarrow
\text{HybridPowerSystem}.
$$

The first exact scope is the balanced AC algebraic network:

- buses and slack/source references,
- series lines with both engineering-unit and per-unit impedance recovery,
- transformer equivalents with nameplate voltage/rating metadata,
- constant-power loads and PQ/static generators.

This bidirectional conversion is the foundation for point-by-point verification:
each imported external component can be projected into the canonical network,
solved by the HACDCPF power-flow engine, exported again, and compared against an
external engine. Later transient verification should extend the same contract to
phase-domain lines, regulator controls, DER controller blocks, and replayable
contingency/event definitions before claiming DAE-level equivalence.

---

# 1. Existing Static Hybrid AC/DC Power-Flow Formulation

Your current module solves a static hybrid AC/DC power flow. For the AC system, the standard positive-sequence or equivalent single-phase formulation is:

$$
S_i = P_i + jQ_i = V_i I_i^*
$$

The injected current at bus $$i$$ is:

$$
I_i = \sum_{j=1}^{N_{ac}} Y_{ij} V_j
$$

Therefore:

$$
S_i = V_i \left(\sum_{j=1}^{N_{ac}} Y_{ij} V_j\right)^*
$$

Let:

$$
V_i = |V_i| e^{j\theta_i}
$$

and:

$$
Y_{ij} = G_{ij} + jB_{ij}
$$

Then the active and reactive power injections are:

$$
P_i =
|V_i|
\sum_{j=1}^{N_{ac}}
|V_j|
\left[
G_{ij}\cos(\theta_i-\theta_j)
+
B_{ij}\sin(\theta_i-\theta_j)
\right]
$$

$$
Q_i =
|V_i|
\sum_{j=1}^{N_{ac}}
|V_j|
\left[
G_{ij}\sin(\theta_i-\theta_j)
-
B_{ij}\cos(\theta_i-\theta_j)
\right]
$$

The AC power-flow mismatch equations are:

$$
\Delta P_i =
P_i^{spec} - P_i^{calc}
$$

$$
\Delta Q_i =
Q_i^{spec} - Q_i^{calc}
$$

For DC networks, with resistive conductance matrix $$G_{dc}$$, the nodal current balance is:

$$
I_k^{dc} =
\sum_{m=1}^{N_{dc}}
G_{km}^{dc} V_m^{dc}
$$

The DC power injection is:

$$
P_k^{dc}
=
V_k^{dc} I_k^{dc}
=
V_k^{dc}
\sum_{m=1}^{N_{dc}}
G_{km}^{dc} V_m^{dc}
$$

Thus:

$$
\Delta P_k^{dc}
=
P_k^{dc,spec}
-
P_k^{dc,calc}
$$

Your static hybrid Newton system is generally:

$$
F(z)=0
$$

with:

$$
z =
\begin{bmatrix}
\theta \\
|V| \\
V^{dc}
\end{bmatrix}
$$

and:

$$
F(z)
=
\begin{bmatrix}
\Delta P^{ac} \\
\Delta Q^{ac} \\
\Delta P^{dc}
\end{bmatrix}
$$

Newton iteration solves:

$$
J(z^{r})\Delta z^{r} = -F(z^{r})
$$

$$
z^{r+1}=z^{r}+\Delta z^{r}
$$

This is static. To model transients, these algebraic equations must be coupled with dynamic device equations.

---

# 2. Three-Phase Unbalanced Network Formulation

For distribution systems, the network should be represented explicitly in phase coordinates.

For each AC bus $$i$$, define the three-phase voltage vector:

$$
\mathbf{v}_i^{abc}
=
\begin{bmatrix}
V_i^a \\
V_i^b \\
V_i^c
\end{bmatrix}
$$

and the current injection vector:

$$
\mathbf{i}_i^{abc}
=
\begin{bmatrix}
I_i^a \\
I_i^b \\
I_i^c
\end{bmatrix}
$$

For the entire AC network:

$$
\mathbf{v}^{abc}
=
\begin{bmatrix}
\mathbf{v}_1^{abc} \\
\mathbf{v}_2^{abc} \\
\vdots \\
\mathbf{v}_{N_{ac}}^{abc}
\end{bmatrix}
$$

$$
\mathbf{i}^{abc}
=
\begin{bmatrix}
\mathbf{i}_1^{abc} \\
\mathbf{i}_2^{abc} \\
\vdots \\
\mathbf{i}_{N_{ac}}^{abc}
\end{bmatrix}
$$

The three-phase network equation is:

$$
\mathbf{i}^{abc}
=
\mathbf{Y}_{abc}
\mathbf{v}^{abc}
$$

where:

$$
\mathbf{Y}_{abc}
\in
\mathbb{C}^{3N_{ac}\times 3N_{ac}}
$$

contains self and mutual admittances.

For a line between buses $$i$$ and $$j$$, the phase impedance matrix is:

$$
\mathbf{Z}_{ij}^{abc}
=
\begin{bmatrix}
z_{aa} & z_{ab} & z_{ac} \\
z_{ba} & z_{bb} & z_{bc} \\
z_{ca} & z_{cb} & z_{cc}
\end{bmatrix}
$$

The series admittance matrix is:

$$
\mathbf{Y}_{ij}^{series}
=
\left(\mathbf{Z}_{ij}^{abc}\right)^{-1}
$$

The line current from bus $$i$$ to bus $$j$$ is:

$$
\mathbf{i}_{ij}^{abc}
=
\mathbf{Y}_{ij}^{series}
\left(
\mathbf{v}_i^{abc}
-
\mathbf{v}_j^{abc}
\right)
+
\mathbf{Y}_{ij}^{sh,i}
\mathbf{v}_i^{abc}
$$

The network admittance matrix is assembled from these primitive branch admittances.

The nodal equation becomes:

$$
\mathbf{Y}_{abc}\mathbf{v}^{abc}
-
\mathbf{i}_{inj}^{abc}
=
0
$$

Therefore, the AC algebraic constraint is:

$$
\mathbf{g}_{ac}
=
\mathbf{Y}_{abc}\mathbf{v}^{abc}
-
\mathbf{i}_{inj}^{abc}(x,\mathbf{v}^{abc},u,t)
=
0
$$

This equation replaces the balanced positive-sequence AC power-flow equation for transient distribution simulation.

---

# 3. Symmetrical-Component Transformation

Although the network is solved in full $$abc$$ coordinates, positive, negative, and zero sequence components are useful for machine and converter controls.

Define:

$$
a = e^{j\frac{2\pi}{3}}
=
-\frac{1}{2}
+
j\frac{\sqrt{3}}{2}
$$

The transformation from sequence to phase is:

$$
\mathbf{v}^{abc}
=
\mathbf{A}
\mathbf{v}^{012}
$$

where:

$$
\mathbf{A}
=
\begin{bmatrix}
1 & 1 & 1 \\
1 & a^2 & a \\
1 & a & a^2
\end{bmatrix}
$$

and:

$$
\mathbf{v}^{012}
=
\begin{bmatrix}
V^0 \\
V^1 \\
V^2
\end{bmatrix}
$$

The inverse transformation is:

$$
\mathbf{v}^{012}
=
\frac{1}{3}
\begin{bmatrix}
1 & 1 & 1 \\
1 & a & a^2 \\
1 & a^2 & a
\end{bmatrix}
\mathbf{v}^{abc}
$$

The positive-sequence voltage is:

$$
V^1
=
\frac{1}{3}
\left(
V^a
+
aV^b
+
a^2V^c
\right)
$$

The negative-sequence voltage is:

$$
V^2
=
\frac{1}{3}
\left(
V^a
+
a^2V^b
+
aV^c
\right)
$$

The zero-sequence voltage is:

$$
V^0
=
\frac{1}{3}
\left(
V^a+V^b+V^c
\right)
$$

This is useful because many machines and inverter controls are internally symmetric and primarily controlled by the positive-sequence component.

---

# 4. General Transient DAE Formulation

The complete hybrid AC/DC transient model is written as:

$$
\dot{x}=f(x,y,u,t)
$$

$$
0=g(x,y,u,t)
$$

where:

$$
x =
\begin{bmatrix}
x_{sg} \\
x_{gov} \\
x_{avr} \\
x_{gfl} \\
x_{gfm} \\
x_{vsc} \\
x_{dcdc} \\
x_{bat} \\
x_{pv} \\
x_{load}
\end{bmatrix}
$$

and:

$$
y =
\begin{bmatrix}
\mathbf{v}^{abc} \\
\mathbf{v}^{dc} \\
\mathbf{i}_{conv}^{abc} \\
\mathbf{i}_{conv}^{dc}
\end{bmatrix}
$$

The algebraic equations are:

$$
g(x,y,u,t)
=
\begin{bmatrix}
g_{ac}(x,y,u,t) \\
g_{dc}(x,y,u,t) \\
g_{conv}(x,y,u,t) \\
g_{lim}(x,y,u,t)
\end{bmatrix}
=
0
$$

The AC network equation is:

$$
g_{ac}
=
\mathbf{Y}_{abc}\mathbf{v}^{abc}
-
\mathbf{i}_{inj}^{abc}
=
0
$$

The DC network equation is:

$$
g_{dc}
=
\mathbf{G}_{dc}\mathbf{v}^{dc}
-
\mathbf{i}_{inj}^{dc}
=
0
$$

The converter coupling equations enforce AC/DC power balance:

$$
P_{conv,ac}
+
P_{conv,dc}
+
P_{loss}
=
0
$$

The dynamic equations describe internal states:

$$
\dot{x}_{device}
=
f_{device}(x_{device},y,u,t)
$$

---

# 5. Phasor-Domain Dynamic Network Solution

In RMS or phasor-domain transient simulation, electrical algebraic variables are updated at each simulation time step.

At time $$t_n$$:

$$
x_n \approx x(t_n)
$$

The dynamic devices are represented by network-equivalent injections:

$$
\mathbf{i}_{inj}^{abc}
=
\sum_d
\mathbf{i}_{d}^{abc}
$$

For device $$d$$, use a Norton form:

$$
\mathbf{i}_{d}^{abc}
=
\mathbf{i}_{N,d}^{abc}(x_d,u,t)
-
\mathbf{Y}_{N,d}^{abc}\mathbf{v}_{d}^{abc}
$$

Then the AC network equation becomes:

$$
\left(
\mathbf{Y}_{abc}
+
\sum_d
\mathbf{Y}_{N,d}^{abc}
\right)
\mathbf{v}^{abc}
=
\sum_d
\mathbf{i}_{N,d}^{abc}
$$

Define the augmented admittance matrix:

$$
\mathbf{Y}_{sys}^{dyn}
=
\mathbf{Y}_{abc}
+
\sum_d
\mathbf{Y}_{N,d}^{abc}
$$

and the total Norton current:

$$
\mathbf{i}_{N}^{tot}
=
\sum_d
\mathbf{i}_{N,d}^{abc}
$$

Therefore:

$$
\mathbf{Y}_{sys}^{dyn}
\mathbf{v}^{abc}
=
\mathbf{i}_{N}^{tot}
$$

This is the core idea of the GridLAB-D-style dynamic interface.

---

# 6. Synchronous Machine Dynamic Derivation

## 6.1 Rotor Reference Frame

For a synchronous generator, define:

- $$\delta$$: rotor electrical angle,
- $$\omega$$: rotor speed in per-unit,
- $$\omega_b$$: base electrical angular frequency,
- $$H$$: inertia constant,
- $$D$$: damping coefficient.

The rotor angle dynamics are:

$$
\dot{\delta}
=
\omega_b(\omega-1)
$$

The swing equation is:

$$
2H\dot{\omega}
=
T_m
-
T_e
-
D(\omega-1)
$$

Equivalently, using power instead of torque:

$$
2H\dot{\omega}
=
\frac{P_m-P_e}{\omega}
-
D(\omega-1)
$$

For small deviations where $$\omega \approx 1$$:

$$
2H\dot{\omega}
=
P_m
-
P_e
-
D(\omega-1)
$$

---

## 6.2 Park Transformation

The machine equations are written in the $$dq0$$ rotor frame.

The phase-to-rotor transformation is:

$$
\begin{bmatrix}
x_d \\
x_q \\
x_0
\end{bmatrix}
=
\frac{2}{3}
\begin{bmatrix}
\cos\delta & \cos(\delta-\frac{2\pi}{3}) & \cos(\delta+\frac{2\pi}{3}) \\
-\sin\delta & -\sin(\delta-\frac{2\pi}{3}) & -\sin(\delta+\frac{2\pi}{3}) \\
\frac{1}{2} & \frac{1}{2} & \frac{1}{2}
\end{bmatrix}
\begin{bmatrix}
x_a \\
x_b \\
x_c
\end{bmatrix}
$$

Denote:

$$
\mathbf{x}_{dq0}
=
\mathbf{T}_{dq0}(\delta)
\mathbf{x}_{abc}
$$

and:

$$
\mathbf{x}_{abc}
=
\mathbf{T}_{abc}(\delta)
\mathbf{x}_{dq0}
$$

The terminal voltage and current in rotor coordinates are:

$$
\mathbf{v}_{dq0}
=
\mathbf{T}_{dq0}(\delta)
\mathbf{v}_{abc}
$$

$$
\mathbf{i}_{dq0}
=
\mathbf{T}_{dq0}(\delta)
\mathbf{i}_{abc}
$$

---

## 6.3 Stator Algebraic Equations

A common subtransient synchronous-machine model is:

$$
v_d =
-r_s i_d
+
x_q'' i_q
+
E_d''
$$

$$
v_q =
-r_s i_q
-
x_d'' i_d
+
E_q''
$$

Equivalently:

$$
\begin{bmatrix}
v_d \\
v_q
\end{bmatrix}
=
\begin{bmatrix}
-r_s & x_q'' \\
-x_d'' & -r_s
\end{bmatrix}
\begin{bmatrix}
i_d \\
i_q
\end{bmatrix}
+
\begin{bmatrix}
E_d'' \\
E_q''
\end{bmatrix}
$$

Rearrange to solve for current:

$$
\begin{bmatrix}
i_d \\
i_q
\end{bmatrix}
=
\mathbf{Z}_{dq}^{-1}
\left(
\begin{bmatrix}
v_d \\
v_q
\end{bmatrix}
-
\begin{bmatrix}
E_d'' \\
E_q''
\end{bmatrix}
\right)
$$

where:

$$
\mathbf{Z}_{dq}
=
\begin{bmatrix}
-r_s & x_q'' \\
-x_d'' & -r_s
\end{bmatrix}
$$

This can be converted to Norton form.

Let:

$$
\mathbf{i}_{dq}
=
\mathbf{Y}_{dq}\mathbf{v}_{dq}
-
\mathbf{Y}_{dq}\mathbf{E}_{dq}''
$$

where:

$$
\mathbf{Y}_{dq}
=
\mathbf{Z}_{dq}^{-1}
$$

and:

$$
\mathbf{E}_{dq}''
=
\begin{bmatrix}
E_d'' \\
E_q''
\end{bmatrix}
$$

In phase coordinates:

$$
\mathbf{i}_{abc}
=
\mathbf{T}_{abc}(\delta)
\mathbf{i}_{dq0}
$$

Thus:

$$
\mathbf{i}_{abc}
=
\mathbf{Y}_{sg}^{abc}(\delta)
\mathbf{v}_{abc}
+
\mathbf{i}_{N,sg}^{abc}(\delta,E_d'',E_q'')
$$

where:

$$
\mathbf{Y}_{sg}^{abc}
=
\mathbf{T}_{abc}
\mathbf{Y}_{dq0}
\mathbf{T}_{dq0}
$$

and:

$$
\mathbf{i}_{N,sg}^{abc}
=
-
\mathbf{T}_{abc}
\mathbf{Y}_{dq0}
\mathbf{E}_{dq0}''
$$

Depending on injection-current sign convention, this may appear as:

$$
\mathbf{i}_{inj,sg}^{abc}
=
\mathbf{i}_{N,sg}^{abc}
-
\mathbf{Y}_{sg}^{abc}
\mathbf{v}_{abc}
$$

The sign should be chosen consistently with your system convention: **positive injection into the network**.

---

## 6.4 Transient and Subtransient Voltage Dynamics

A sixth-order synchronous-machine model uses:

$$
\dot{E}_q'
=
\frac{
E_{fd}
-
E_q'
-
(x_d-x_d')i_d
}
{T_{do}'}
$$

$$
\dot{E}_d'
=
\frac{
-
E_d'
+
(x_q-x_q')i_q
}
{T_{qo}'}
$$

$$
\dot{E}_q''
=
\frac{
E_q'
-
E_q''
-
(x_d'-x_d'')i_d
}
{T_{do}''}
$$

$$
\dot{E}_d''
=
\frac{
E_d'
-
E_d''
+
(x_q'-x_q'')i_q
}
{T_{qo}''}
$$

where:

- $$E_q'$$, $$E_d'$$ are transient internal voltages,
- $$E_q''$$, $$E_d''$$ are subtransient internal voltages,
- $$x_d$$, $$x_q$$ are synchronous reactances,
- $$x_d'$$, $$x_q'$$ are transient reactances,
- $$x_d''$$, $$x_q''$$ are subtransient reactances,
- $$T_{do}'$$, $$T_{qo}'$$ are transient open-circuit time constants,
- $$T_{do}''$$, $$T_{qo}''$$ are subtransient open-circuit time constants.

---

## 6.5 Electrical Power and Torque Under Unbalance

The instantaneous three-phase complex power is:

$$
S_{abc}
=
\sum_{\phi \in \{a,b,c\}}
V_{\phi} I_{\phi}^*
$$

The active electrical power is:

$$
P_e
=
\operatorname{Re}
\left(
V_a I_a^*
+
V_b I_b^*
+
V_c I_c^*
\right)
$$

The electromagnetic torque is:

$$
T_e
=
\frac{P_e}{\omega}
$$

Under unbalanced operation, negative-sequence currents produce torque ripple at approximately twice fundamental frequency. In electromechanical phasor simulation, this high-frequency torque ripple is averaged.

Therefore, use:

$$
\overline{T}_e
=
\frac{\overline{P}_e}{\omega}
$$

where:

$$
\overline{P}_e
=
\operatorname{Re}
\left(
V_a I_a^*
+
V_b I_b^*
+
V_c I_c^*
\right)
$$

using the fundamental-frequency phasors.

The swing equation becomes:

$$
2H\dot{\omega}
=
T_m
-
\overline{T}_e
-
D(\omega-1)
$$

This is the key approximation used by the GridLAB-D paper.

---

# 7. Governor Model

A simple turbine-governor model can be written as droop plus first-order lag.

Let:

- $$P_m$$ be mechanical power,
- $$P_{ref}$$ be mechanical power reference,
- $$R$$ be speed droop,
- $$T_g$$ be governor time constant.

The governor command is:

$$
P_g^{cmd}
=
P_{ref}
-
\frac{1}{R}(\omega-1)
$$

The mechanical power dynamics are:

$$
T_g\dot{P}_m
=
P_g^{cmd}
-
P_m
$$

or:

$$
\dot{P}_m
=
\frac{1}{T_g}
\left[
P_{ref}
-
\frac{1}{R}(\omega-1)
-
P_m
\right]
$$

For isochronous control, the droop term can be replaced by integral frequency control:

$$
\dot{\xi}_{iso}
=
\omega_{ref}
-
\omega
$$

$$
P_g^{cmd}
=
P_{ref}
+
K_p(\omega_{ref}-\omega)
+
K_i\xi_{iso}
$$

Then:

$$
\dot{P}_m
=
\frac{P_g^{cmd}-P_m}{T_g}
$$

---

# 8. Exciter and AVR Model

A simple automatic voltage regulator can be modeled as:

$$
\dot{\xi}_{v}
=
V_{ref}
-
V_t
$$

$$
E_{fd}^{cmd}
=
K_A(V_{ref}-V_t)
+
K_I\xi_v
$$

A first-order exciter is:

$$
T_A\dot{E}_{fd}
=
-E_{fd}
+
E_{fd}^{cmd}
$$

Thus:

$$
\dot{E}_{fd}
=
\frac{
-E_{fd}
+
K_A(V_{ref}-V_t)
+
K_I\xi_v
}
{T_A}
$$

The terminal voltage magnitude may be computed from positive sequence:

$$
V_t
=
|V^1|
$$

or from RMS phase average:

$$
V_t
=
\sqrt{
\frac{
|V_a|^2+|V_b|^2+|V_c|^2
}{3}
}
$$

For distribution unbalance studies, the positive-sequence voltage is often preferable for machine controls:

$$
V_t = |V^1|
$$

---

# 9. Grid-Following Inverter Dynamic Derivation

Grid-following inverters are current-controlled devices synchronized to the grid using a PLL.

They are appropriate for:

- PV systems,
- most battery systems,
- PQ-controlled VSCs,
- renewable generators,
- EV chargers.

---

## 9.1 PLL Model

Let the measured positive-sequence AC voltage be transformed to the PLL reference frame:

$$
\begin{bmatrix}
v_d^{pll} \\
v_q^{pll}
\end{bmatrix}
=
\mathbf{T}_{dq}(\theta_{pll})
\mathbf{v}_{abc}^{+}
$$

The PLL tries to force:

$$
v_q^{pll} \rightarrow 0
$$

A standard PI PLL is:

$$
\dot{\xi}_{pll}
=
v_q^{pll}
$$

$$
\omega_{pll}
=
\omega_0
+
K_{p,pll}v_q^{pll}
+
K_{i,pll}\xi_{pll}
$$

$$
\dot{\theta}_{pll}
=
\omega_{pll}
$$

Thus, the PLL states are:

$$
x_{pll}
=
\begin{bmatrix}
\theta_{pll} \\
\xi_{pll}
\end{bmatrix}
$$

---

## 9.2 Power Measurement

In the PLL reference frame:

$$
P
=
v_d i_d
+
v_q i_q
$$

$$
Q
=
v_q i_d
-
v_d i_q
$$

If the PLL is aligned so that:

$$
v_q \approx 0
$$

then:

$$
P \approx v_d i_d
$$

$$
Q \approx -v_d i_q
$$

Therefore:

$$
i_d^{ref}
\approx
\frac{P^{ref}}{v_d}
$$

$$
i_q^{ref}
\approx
-\frac{Q^{ref}}{v_d}
$$

More generally:

$$
\begin{bmatrix}
P^{ref} \\
Q^{ref}
\end{bmatrix}
=
\begin{bmatrix}
v_d & v_q \\
v_q & -v_d
\end{bmatrix}
\begin{bmatrix}
i_d^{ref} \\
i_q^{ref}
\end{bmatrix}
$$

Hence:

$$
\begin{bmatrix}
i_d^{ref} \\
i_q^{ref}
\end{bmatrix}
=
\frac{1}{v_d^2+v_q^2}
\begin{bmatrix}
v_d & v_q \\
v_q & -v_d
\end{bmatrix}
\begin{bmatrix}
P^{ref} \\
Q^{ref}
\end{bmatrix}
$$

---

## 9.3 Current Controller Dynamics

Use a first-order approximation of the inner current loop:

$$
T_i \dot{i}_d
=
-i_d
+
i_d^{ref}
$$

$$
T_i \dot{i}_q
=
-i_q
+
i_q^{ref}
$$

Thus:

$$
\dot{i}_d
=
\frac{i_d^{ref}-i_d}{T_i}
$$

$$
\dot{i}_q
=
\frac{i_q^{ref}-i_q}{T_i}
$$

---

## 9.4 Current Limiting

Converter current magnitude is limited:

$$
i_d^2+i_q^2
\leq
I_{max}^2
$$

If:

$$
\sqrt{
(i_d^{ref})^2+(i_q^{ref})^2
}
\leq
I_{max}
$$

then no limiting is required.

If the limit is violated, define:

$$
\lambda
=
\frac{
I_{max}
}{
\sqrt{
(i_d^{ref})^2+(i_q^{ref})^2
}
}
$$

and set:

$$
i_d^{lim}
=
\lambda i_d^{ref}
$$

$$
i_q^{lim}
=
\lambda i_q^{ref}
$$

For reactive-current priority, during voltage sag:

$$
i_q^{lim}
=
\operatorname{sat}
\left(
i_q^{ref},
-I_{max},
I_{max}
\right)
$$

$$
i_d^{lim}
=
\operatorname{sgn}(i_d^{ref})
\sqrt{
I_{max}^2
-
(i_q^{lim})^2
}
$$

---

## 9.5 Current Injection into Three-Phase Network

The inverter current in the PLL frame is:

$$
\mathbf{i}_{dq}
=
\begin{bmatrix}
i_d \\
i_q
\end{bmatrix}
$$

Convert to phase coordinates:

$$
\mathbf{i}_{abc}^{inv}
=
\mathbf{T}_{abc}(\theta_{pll})
\begin{bmatrix}
i_d \\
i_q \\
0
\end{bmatrix}
$$

Thus, the grid-following inverter is represented as a controlled current source:

$$
\mathbf{i}_{inj,gfl}^{abc}
=
\mathbf{i}_{abc}^{inv}
$$

Optionally, numerical stabilization can add a small parallel admittance:

$$
\mathbf{i}_{inj,gfl}^{abc}
=
\mathbf{i}_{abc}^{inv}
-
\mathbf{Y}_{stab}
\mathbf{v}_{abc}
$$

---

# 10. Grid-Forming Inverter Dynamic Derivation

Grid-forming inverters regulate voltage magnitude and frequency directly.

They are appropriate for:

- islanded microgrids,
- black-start systems,
- battery energy storage,
- converter-dominated networks.

---

## 10.1 Droop Control

Let:

- $$P$$ be measured active power,
- $$Q$$ be measured reactive power,
- $$P^{ref}$$ and $$Q^{ref}$$ be references,
- $$\omega_0$$ be nominal angular frequency,
- $$V_0$$ be nominal voltage magnitude,
- $$m_p$$ be active-power frequency droop,
- $$n_q$$ be reactive-power voltage droop.

Frequency command:

$$
\omega
=
\omega_0
-
m_p(P-P^{ref})
$$

Voltage command:

$$
E
=
V_0
-
n_q(Q-Q^{ref})
$$

The internal voltage angle evolves as:

$$
\dot{\theta}
=
\omega
$$

Therefore:

$$
\dot{\theta}
=
\omega_0
-
m_p(P-P^{ref})
$$

A filtered power measurement is commonly used:

$$
T_p\dot{P}_f
=
P-P_f
$$

$$
T_q\dot{Q}_f
=
Q-Q_f
$$

Then:

$$
\dot{\theta}
=
\omega_0
-
m_p(P_f-P^{ref})
$$

$$
E
=
V_0
-
n_q(Q_f-Q^{ref})
$$

---

## 10.2 Voltage Source Behind Virtual Impedance

The grid-forming inverter is modeled as an internal voltage source behind virtual impedance.

Let:

$$
\mathbf{e}_{abc}
=
E
\begin{bmatrix}
e^{j\theta} \\
e^{j(\theta-\frac{2\pi}{3})} \\
e^{j(\theta+\frac{2\pi}{3})}
\end{bmatrix}
$$

Let the virtual impedance be:

$$
\mathbf{Z}_v^{abc}
=
\begin{bmatrix}
Z_v & 0 & 0 \\
0 & Z_v & 0 \\
0 & 0 & Z_v
\end{bmatrix}
$$

where:

$$
Z_v = R_v + jX_v
$$

The admittance is:

$$
\mathbf{Y}_v^{abc}
=
\left(
\mathbf{Z}_v^{abc}
\right)^{-1}
$$

The output current is:

$$
\mathbf{i}_{gfm}^{abc}
=
\mathbf{Y}_v^{abc}
\left(
\mathbf{e}_{abc}
-
\mathbf{v}_{abc}
\right)
$$

Thus:

$$
\mathbf{i}_{gfm}^{abc}
=
\mathbf{Y}_v^{abc}\mathbf{e}_{abc}
-
\mathbf{Y}_v^{abc}\mathbf{v}_{abc}
$$

This is Norton form with:

$$
\mathbf{i}_{N,gfm}^{abc}
=
\mathbf{Y}_v^{abc}\mathbf{e}_{abc}
$$

and:

$$
\mathbf{Y}_{N,gfm}^{abc}
=
\mathbf{Y}_v^{abc}
$$

So the injection equation is:

$$
\mathbf{i}_{inj,gfm}^{abc}
=
\mathbf{i}_{N,gfm}^{abc}
-
\mathbf{Y}_{N,gfm}^{abc}
\mathbf{v}_{abc}
$$

This is directly compatible with the synchronous-machine Norton interface.

---

## 10.3 Virtual Synchronous Machine Model

An alternative grid-forming model is the virtual synchronous machine.

The virtual swing equation is:

$$
2H_v\dot{\omega}
=
P_m^{v}
-
P_e
-
D_v(\omega-\omega_0)
$$

The angle equation is:

$$
\dot{\theta}
=
\omega
$$

The internal voltage can be controlled by a virtual exciter:

$$
T_E\dot{E}
=
-E
+
E^{ref}
+
K_v(V^{ref}-V_t)
$$

Then the network injection is still:

$$
\mathbf{i}_{gfm}^{abc}
=
\mathbf{Y}_v^{abc}
\left(
\mathbf{e}_{abc}
-
\mathbf{v}_{abc}
\right)
$$

---

# 11. VSC AC/DC Converter Dynamic Derivation

A VSC couples an AC bus and a DC bus.

The static power balance is:

$$
P_{ac}
+
P_{dc}
+
P_{loss}
=
0
$$

In transient simulation, this is extended by converter controls and DC-link dynamics.

---

## 11.1 AC-Side Power

For a three-phase AC terminal:

$$
S_{ac}
=
P_{ac}+jQ_{ac}
=
V_a I_a^*
+
V_b I_b^*
+
V_c I_c^*
$$

Thus:

$$
P_{ac}
=
\operatorname{Re}
\left(
V_a I_a^*
+
V_b I_b^*
+
V_c I_c^*
\right)
$$

$$
Q_{ac}
=
\operatorname{Im}
\left(
V_a I_a^*
+
V_b I_b^*
+
V_c I_c^*
\right)
$$

Sign convention:

- positive $$P_{ac}$$ means injection into AC network,
- positive $$P_{dc}$$ means injection into DC network.

Converter loss requires:

$$
P_{ac}
+
P_{dc}
+
P_{loss}
=
0
$$

---

## 11.2 Loss Model

A common VSC loss model is:

$$
P_{loss}
=
a
+
b I_{conv}
+
c I_{conv}^2
$$

where:

$$
I_{conv}
=
\sqrt{
|I_a|^2+|I_b|^2+|I_c|^2
}
$$

or in dq form:

$$
I_{conv}
=
\sqrt{
i_d^2+i_q^2
}
$$

---

## 11.3 DC-Link Capacitor Dynamics

Let the converter DC link have capacitance $$C_{dc}$$ and voltage $$V_{dc}$$.

The stored energy is:

$$
W_{dc}
=
\frac{1}{2}C_{dc}V_{dc}^2
$$

Energy balance gives:

$$
\frac{dW_{dc}}{dt}
=
P_{dc,in}
-
P_{ac,out}
-
P_{loss}
$$

Since:

$$
\frac{dW_{dc}}{dt}
=
C_{dc}V_{dc}\dot{V}_{dc}
$$

we have:

$$
C_{dc}V_{dc}\dot{V}_{dc}
=
P_{dc,in}
-
P_{ac,out}
-
P_{loss}
$$

Using bus-injection convention, a convenient form is:

$$
C_{dc}V_{dc}\dot{V}_{dc}
=
-
P_{ac}
-
P_{dc}
-
P_{loss}
$$

If the algebraic converter power balance is exact, then:

$$
P_{ac}
+
P_{dc}
+
P_{loss}
=
0
$$

and the DC-link energy state is quasi-steady. If dynamic energy imbalance is allowed, then:

$$
\dot{V}_{dc}
=
\frac{
-P_{ac}
-
P_{dc}
-
P_{loss}
}{
C_{dc}V_{dc}
}
$$

This equation is essential for transient hybrid AC/DC studies.

---

## 11.4 Vdc-Q Control Mode

Suppose the VSC regulates DC voltage and AC reactive power.

The DC voltage controller is:

$$
e_{vdc}
=
V_{dc}^{ref}
-
V_{dc}
$$

$$
\dot{\xi}_{vdc}
=
e_{vdc}
$$

The active power reference is:

$$
P_{ac}^{ref}
=
K_{p,vdc}e_{vdc}
+
K_{i,vdc}\xi_{vdc}
$$

The reactive power reference is:

$$
Q_{ac}^{ref}
=
Q^{ref}
$$

Then the grid-following current references are:

$$
\begin{bmatrix}
i_d^{ref} \\
i_q^{ref}
\end{bmatrix}
=
\frac{1}{v_d^2+v_q^2}
\begin{bmatrix}
v_d & v_q \\
v_q & -v_d
\end{bmatrix}
\begin{bmatrix}
P_{ac}^{ref} \\
Q_{ac}^{ref}
\end{bmatrix}
$$

---

## 11.5 Vdc-Vac Control Mode

For DC voltage and AC voltage regulation:

$$
e_{vdc}
=
V_{dc}^{ref}
-
V_{dc}
$$

$$
\dot{\xi}_{vdc}
=
e_{vdc}
$$

$$
P_{ac}^{ref}
=
K_{p,vdc}e_{vdc}
+
K_{i,vdc}\xi_{vdc}
$$

For AC voltage:

$$
e_{vac}
=
V_{ac}^{ref}
-
V_{ac}
$$

$$
\dot{\xi}_{vac}
=
e_{vac}
$$

$$
Q_{ac}^{ref}
=
K_{p,vac}e_{vac}
+
K_{i,vac}\xi_{vac}
$$

Then the current references are again obtained from the power-current mapping.

---

# 12. DC Network Dynamic Formulation

Your DC network is resistive in the static model:

$$
\mathbf{i}^{dc}
=
\mathbf{G}_{dc}\mathbf{v}^{dc}
$$

For transient simulation, DC buses with capacitance can be modeled dynamically.

At DC bus $$k$$, let capacitance be $$C_k^{dc}$$. The nodal current balance is:

$$
C_k^{dc}\dot{V}_k^{dc}
=
I_{inj,k}^{dc}
-
\sum_{m=1}^{N_{dc}}
G_{km}^{dc}V_m^{dc}
$$

For all DC buses:

$$
\mathbf{C}_{dc}\dot{\mathbf{v}}^{dc}
=
\mathbf{i}_{inj}^{dc}
-
\mathbf{G}_{dc}\mathbf{v}^{dc}
$$

where:

$$
\mathbf{C}_{dc}
=
\operatorname{diag}
\left(
C_1^{dc},C_2^{dc},\dots,C_{N_{dc}}^{dc}
\right)
$$

If DC capacitances are neglected, this reduces to the algebraic equation:

$$
\mathbf{G}_{dc}\mathbf{v}^{dc}
-
\mathbf{i}_{inj}^{dc}
=
0
$$

Therefore, you have two modeling choices:

## Algebraic DC network

$$
0
=
\mathbf{G}_{dc}\mathbf{v}^{dc}
-
\mathbf{i}_{inj}^{dc}
$$

## Dynamic DC network

$$
\dot{\mathbf{v}}^{dc}
=
\mathbf{C}_{dc}^{-1}
\left(
\mathbf{i}_{inj}^{dc}
-
\mathbf{G}_{dc}\mathbf{v}^{dc}
\right)
$$

For hybrid AC/DC transient simulation, the second form is more physically meaningful when DC capacitors are important.

---

# 13. DC/DC Converter Dynamic Derivation

A DC/DC converter connects DC buses $$k$$ and $$m$$.

Let:

- input-side voltage be $$V_k^{dc}$$,
- output-side voltage be $$V_m^{dc}$$,
- input power be $$P_{in}$$,
- output power be $$P_{out}$$,
- efficiency be $$\eta$$.

For converter operation from bus $$k$$ to bus $$m$$:

$$
P_{out}
=
\eta P_{in}
$$

The current injections are:

$$
I_k^{dc}
=
-\frac{P_{in}}{V_k^{dc}}
$$

$$
I_m^{dc}
=
\frac{P_{out}}{V_m^{dc}}
$$

Using:

$$
P_{in}
=
\frac{P_{out}}{\eta}
$$

we have:

$$
I_k^{dc}
=
-\frac{P_{out}}{\eta V_k^{dc}}
$$

$$
I_m^{dc}
=
\frac{P_{out}}{V_m^{dc}}
$$

A first-order power controller is:

$$
T_{dcdc}\dot{P}_{out}
=
-P_{out}
+
P_{out}^{ref}
$$

Thus:

$$
\dot{P}_{out}
=
\frac{
P_{out}^{ref}
-
P_{out}
}{
T_{dcdc}
}
$$

For voltage regulation at output bus:

$$
e_v
=
V_m^{dc,ref}
-
V_m^{dc}
$$

$$
\dot{\xi}_v
=
e_v
$$

$$
P_{out}^{ref}
=
K_{p,dcdc}e_v
+
K_{i,dcdc}\xi_v
$$

Current limits are:

$$
\left|
\frac{P_{in}}{V_k^{dc}}
\right|
\leq
I_{k,max}
$$

$$
\left|
\frac{P_{out}}{V_m^{dc}}
\right|
\leq
I_{m,max}
$$

Therefore:

$$
|P_{out}|
\leq
\min
\left(
\eta V_k^{dc}I_{k,max},
V_m^{dc}I_{m,max}
\right)
$$

---

# 14. Battery Storage Dynamic Model

For battery energy storage, the main slow state is state of charge.

Let:

- $$E_{rated}$$ be rated energy,
- $$SoC$$ be state of charge,
- $$P_{bat}$$ be battery DC power,
- positive $$P_{bat}$$ means discharge.

The ideal SoC equation is:

$$
\dot{SoC}
=
-\frac{P_{bat}}{E_{rated}}
$$

With charge/discharge efficiencies:

For discharge:

$$
P_{bat} > 0
$$

$$
\dot{SoC}
=
-\frac{P_{bat}}{\eta_{dis}E_{rated}}
$$

For charge:

$$
P_{bat} < 0
$$

$$
\dot{SoC}
=
-\frac{\eta_{ch}P_{bat}}{E_{rated}}
$$

This can be written piecewise:

$$
\dot{SoC}
=
\begin{cases}
-\dfrac{P_{bat}}{\eta_{dis}E_{rated}}, & P_{bat}\geq 0 \\
-\dfrac{\eta_{ch}P_{bat}}{E_{rated}}, & P_{bat}<0
\end{cases}
$$

The constraints are:

$$
SoC_{min}
\leq
SoC
\leq
SoC_{max}
$$

$$
-P_{ch,max}
\leq
P_{bat}
\leq
P_{dis,max}
$$

For transient inverter-based battery operation, the battery typically provides the DC-side power for a VSC. Therefore:

$$
P_{dc}
=
P_{bat}
$$

and the VSC determines AC-side injection.

---

# 15. PV Dynamic Model

PV available power depends on irradiance and temperature.

A simplified available-power model is:

$$
P_{mppt}
=
P_{rated}
\frac{G}{G_{STC}}
\left[
1+\alpha_T(T_c-T_{STC})
\right]
$$

where:

- $$G$$ is irradiance,
- $$G_{STC}$$ is standard irradiance,
- $$T_c$$ is cell temperature,
- $$T_{STC}$$ is standard test temperature,
- $$\alpha_T$$ is temperature coefficient.

The active power command is:

$$
P_{pv}^{ref}
=
\min
\left(
P_{mppt},
P_{curt}
\right)
$$

For frequency-watt control:

$$
P_{pv}^{ref}
=
P_{mppt}
-
K_f(\omega-\omega_{db})
$$

when:

$$
\omega > \omega_{db}
$$

For volt-var control:

$$
Q_{pv}^{ref}
=
f_{VV}(V)
$$

A simple linear volt-var characteristic is:

$$
Q_{pv}^{ref}
=
K_{vv}(V^{ref}-V)
$$

subject to the inverter apparent power limit:

$$
(P_{pv}^{ref})^2
+
(Q_{pv}^{ref})^2
\leq
S_{rated}^2
$$

Thus:

$$
|Q_{pv}^{ref}|
\leq
\sqrt{
S_{rated}^2
-
(P_{pv}^{ref})^2
}
$$

The PV inverter is then represented using the grid-following inverter equations.

---

# 16. Dynamic Load Model

## 16.1 ZIP Load

For each phase, a ZIP load is:

$$
P(V)
=
P_0
\left[
a_z\left(\frac{|V|}{V_0}\right)^2
+
a_i\left(\frac{|V|}{V_0}\right)
+
a_p
\right]
$$

$$
Q(V)
=
Q_0
\left[
b_z\left(\frac{|V|}{V_0}\right)^2
+
b_i\left(\frac{|V|}{V_0}\right)
+
b_p
\right]
$$

with:

$$
a_z+a_i+a_p=1
$$

$$
b_z+b_i+b_p=1
$$

The corresponding phase current demand is:

$$
I_{load}
=
\left(
\frac{P+jQ}{V}
\right)^*
$$

Since load is consumption, current injection into the network is:

$$
I_{inj,load}
=
-
I_{load}
=
-
\left(
\frac{P+jQ}{V}
\right)^*
$$

---

## 16.2 Frequency-Dependent Load

A frequency-sensitive load is:

$$
P(V,\omega)
=
P_{ZIP}(V)
\left[
1
+
K_{pf}(\omega-1)
\right]
$$

$$
Q(V,\omega)
=
Q_{ZIP}(V)
\left[
1
+
K_{qf}(\omega-1)
\right]
$$

---

## 16.3 First-Order Load Recovery

For thermostatic or aggregate load recovery:

$$
T_L\dot{P}_L
=
-P_L
+
P_{ss}(V,\omega)
$$

$$
T_L\dot{Q}_L
=
-Q_L
+
Q_{ss}(V,\omega)
$$

The injected current is:

$$
I_{inj,L}
=
-
\left(
\frac{P_L+jQ_L}{V}
\right)^*
$$

---

# 17. Unified Device Norton Interface

Every dynamic AC device should be converted to:

$$
\mathbf{i}_{inj,d}^{abc}
=
\mathbf{i}_{N,d}^{abc}
-
\mathbf{Y}_{N,d}^{abc}
\mathbf{v}_{d}^{abc}
$$

For multiple devices, the total current injection is:

$$
\mathbf{i}_{inj}^{abc}
=
\sum_{d=1}^{N_d}
\mathbf{C}_d^T
\left(
\mathbf{i}_{N,d}^{abc}
-
\mathbf{Y}_{N,d}^{abc}
\mathbf{C}_d\mathbf{v}^{abc}
\right)
$$

where $$\mathbf{C}_d$$ maps global bus voltages to the local terminal voltage of device $$d$$.

Thus:

$$
\mathbf{i}_{inj}^{abc}
=
\sum_d
\mathbf{C}_d^T\mathbf{i}_{N,d}^{abc}
-
\sum_d
\mathbf{C}_d^T
\mathbf{Y}_{N,d}^{abc}
\mathbf{C}_d
\mathbf{v}^{abc}
$$

The network equation is:

$$
\mathbf{Y}_{abc}\mathbf{v}^{abc}
=
\mathbf{i}_{inj}^{abc}
$$

Substitute:

$$
\mathbf{Y}_{abc}\mathbf{v}^{abc}
=
\sum_d
\mathbf{C}_d^T\mathbf{i}_{N,d}^{abc}
-
\sum_d
\mathbf{C}_d^T
\mathbf{Y}_{N,d}^{abc}
\mathbf{C}_d
\mathbf{v}^{abc}
$$

Rearrange:

$$
\left(
\mathbf{Y}_{abc}
+
\sum_d
\mathbf{C}_d^T
\mathbf{Y}_{N,d}^{abc}
\mathbf{C}_d
\right)
\mathbf{v}^{abc}
=
\sum_d
\mathbf{C}_d^T
\mathbf{i}_{N,d}^{abc}
$$

Define:

$$
\mathbf{Y}_{eff}^{abc}
=
\mathbf{Y}_{abc}
+
\sum_d
\mathbf{C}_d^T
\mathbf{Y}_{N,d}^{abc}
\mathbf{C}_d
$$

and:

$$
\mathbf{i}_{eff}^{abc}
=
\sum_d
\mathbf{C}_d^T
\mathbf{i}_{N,d}^{abc}
$$

Then:

$$
\mathbf{Y}_{eff}^{abc}
\mathbf{v}^{abc}
=
\mathbf{i}_{eff}^{abc}
$$

This is the fundamental algebraic solve at each transient time step.

---

# 18. Hybrid AC/DC Coupled Network Formulation

For AC/DC systems, define the algebraic/dynamic network variables:

$$
y
=
\begin{bmatrix}
\mathbf{v}^{abc} \\
\mathbf{v}^{dc}
\end{bmatrix}
$$

The coupled network equations are:

$$
\mathbf{Y}_{eff}^{abc}(x)
\mathbf{v}^{abc}
=
\mathbf{i}_{eff}^{abc}(x,\mathbf{v}^{dc},u,t)
$$

$$
\mathbf{C}_{dc}\dot{\mathbf{v}}^{dc}
=
\mathbf{i}_{inj}^{dc}(x,\mathbf{v}^{abc},\mathbf{v}^{dc},u,t)
-
\mathbf{G}_{dc}\mathbf{v}^{dc}
$$

If DC bus voltages are treated algebraically:

$$
\mathbf{G}_{dc}\mathbf{v}^{dc}
=
\mathbf{i}_{inj}^{dc}(x,\mathbf{v}^{abc},\mathbf{v}^{dc},u,t)
$$

The converter equations provide coupling:

$$
P_{ac,d}
=
\operatorname{Re}
\left(
(\mathbf{v}_{d}^{abc})^H
\mathbf{i}_{d}^{abc}
\right)
$$

where $$H$$ denotes conjugate transpose.

The DC power is:

$$
P_{dc,d}
=
V_{dc,d} I_{dc,d}
$$

The converter energy balance is:

$$
P_{ac,d}
+
P_{dc,d}
+
P_{loss,d}
+
C_{dc,d}V_{dc,d}\dot{V}_{dc,d}
=
0
$$

This is the key hybrid AC/DC transient coupling equation.

---

# 19. Complete State Vector

A complete dynamic state vector can be assembled as:

$$
x =
\begin{bmatrix}
x_{sg,1} \\
\vdots \\
x_{sg,N_{sg}} \\
x_{gfl,1} \\
\vdots \\
x_{gfl,N_{gfl}} \\
x_{gfm,1} \\
\vdots \\
x_{gfm,N_{gfm}} \\
x_{vsc,1} \\
\vdots \\
x_{vsc,N_{vsc}} \\
x_{dcdc,1} \\
\vdots \\
x_{dcdc,N_{dcdc}} \\
x_{bat,1} \\
\vdots \\
x_{bat,N_{bat}} \\
x_{load,1} \\
\vdots \\
x_{load,N_{load}}
\end{bmatrix}
$$

For a synchronous generator:

$$
x_{sg}
=
\begin{bmatrix}
\delta \\
\omega \\
E_q' \\
E_d' \\
E_q'' \\
E_d'' \\
P_m \\
E_{fd}
\end{bmatrix}
$$

For a grid-following inverter:

$$
x_{gfl}
=
\begin{bmatrix}
\theta_{pll} \\
\xi_{pll} \\
i_d \\
i_q \\
\xi_P \\
\xi_Q
\end{bmatrix}
$$

For a grid-forming inverter:

$$
x_{gfm}
=
\begin{bmatrix}
\theta \\
\omega \\
E \\
P_f \\
Q_f
\end{bmatrix}
$$

For a VSC:

$$
x_{vsc}
=
\begin{bmatrix}
\theta_{pll} \\
\xi_{pll} \\
i_d \\
i_q \\
V_{dc} \\
\xi_{vdc} \\
\xi_{vac}
\end{bmatrix}
$$

For a DC/DC converter:

$$
x_{dcdc}
=
\begin{bmatrix}
P_{out} \\
\xi_v
\end{bmatrix}
$$

For a battery:

$$
x_{bat}
=
\begin{bmatrix}
SoC
\end{bmatrix}
$$

For a dynamic load:

$$
x_{load}
=
\begin{bmatrix}
P_L \\
Q_L
\end{bmatrix}
$$

---

# 20. Initialization from Static Power Flow

Transient simulation must start from a consistent operating point.

Your existing PF/OPF provides:

$$
\mathbf{v}_0^{abc}
$$

$$
\mathbf{v}_0^{dc}
$$

$$
P_{g,0},Q_{g,0}
$$

$$
P_{conv,0},Q_{conv,0}
$$

$$
P_{load,0},Q_{load,0}
$$

The initial dynamic states must satisfy:

$$
f(x_0,y_0,u_0,t_0)=0
$$

$$
g(x_0,y_0,u_0,t_0)=0
$$

---

## 20.1 Synchronous Machine Initialization

Given terminal voltage $$V_t$$ and generator power $$S_g=P_g+jQ_g$$:

$$
I_t
=
\left(
\frac{S_g}{V_t}
\right)^*
$$

Transform to rotor frame:

$$
\begin{bmatrix}
i_d \\
i_q
\end{bmatrix}
=
\mathbf{T}_{dq}(\delta)
\begin{bmatrix}
i_a \\
i_b \\
i_c
\end{bmatrix}
$$

The initial internal voltage behind subtransient reactance is:

$$
\mathbf{E}_{dq}''
=
\mathbf{v}_{dq}
-
\mathbf{Z}_{dq}
\mathbf{i}_{dq}
$$

Then:

$$
E_d''(0)
=
\text{first component of }
\mathbf{E}_{dq}''
$$

$$
E_q''(0)
=
\text{second component of }
\mathbf{E}_{dq}''
$$

At steady state:

$$
\dot{\omega}=0
$$

Therefore:

$$
P_m(0)
=
P_e(0)
$$

Also:

$$
\omega(0)=1
$$

and:

$$
\dot{\delta}=0
$$

For transient voltages, enforce:

$$
\dot{E}_q'=0
$$

$$
\dot{E}_d'=0
$$

$$
\dot{E}_q''=0
$$

$$
\dot{E}_d''=0
$$

For example:

$$
0
=
E_{fd}
-
E_q'
-
(x_d-x_d')i_d
$$

so:

$$
E_{fd}(0)
=
E_q'(0)
+
(x_d-x_d')i_d(0)
$$

Similarly:

$$
0
=
-E_d'
+
(x_q-x_q')i_q
$$

so:

$$
E_d'(0)
=
(x_q-x_q')i_q(0)
$$

---

## 20.2 Grid-Following Inverter Initialization

Given:

$$
P_0,Q_0,V_0
$$

Set PLL angle:

$$
\theta_{pll}(0)
=
\angle V^1
$$

Set PLL integrator so:

$$
\omega_{pll}(0)=\omega_0
$$

From the power-current relation:

$$
\begin{bmatrix}
i_d(0) \\
i_q(0)
\end{bmatrix}
=
\frac{1}{v_d^2+v_q^2}
\begin{bmatrix}
v_d & v_q \\
v_q & -v_d
\end{bmatrix}
\begin{bmatrix}
P_0 \\
Q_0
\end{bmatrix}
$$

Set:

$$
i_d^{ref}(0)=i_d(0)
$$

$$
i_q^{ref}(0)=i_q(0)
$$

---

## 20.3 Grid-Forming Inverter Initialization

Given terminal voltage and desired power:

$$
P_0,Q_0
$$

Set:

$$
\theta(0)
=
\angle V^1
$$

$$
P_f(0)=P_0
$$

$$
Q_f(0)=Q_0
$$

From droop equations:

$$
\omega(0)
=
\omega_0
-
m_p(P_0-P^{ref})
$$

For steady nominal frequency:

$$
P^{ref}=P_0
$$

so:

$$
\omega(0)=\omega_0
$$

Similarly:

$$
E(0)
=
V_0
-
n_q(Q_0-Q^{ref})
$$

For steady voltage:

$$
Q^{ref}=Q_0
$$

so:

$$
E(0)=V_0
$$

If modeling as voltage source behind impedance, compute internal voltage:

$$
\mathbf{e}_{abc}(0)
=
\mathbf{v}_{abc}(0)
+
\mathbf{Z}_v^{abc}\mathbf{i}_{abc}(0)
$$

---

# 21. Islanded Microgrid Initialization

In an islanded microgrid, no bus has a fixed external reference. Therefore, the static PF must be modified.

Let there be $$N_G$$ grid-forming sources. These can be:

- synchronous machines,
- grid-forming inverters,
- VSCs in grid-forming mode.

A generalized initialization procedure is:

## Step 1: Choose temporary reference

Select one grid-forming source $$r$$ and impose:

$$
|V_r^1|=V_{ref}
$$

$$
\angle V_r^1=0
$$

This acts as a temporary slack.

## Step 2: Solve unbalanced power flow

Solve:

$$
g_{PF}(y)=0
$$

with:

- source $$r$$ as temporary slack,
- other sources as PQ or droop-equivalent devices.

## Step 3: Convert all sources to dynamic Norton equivalents

For each source $$d$$, compute:

$$
\mathbf{i}_{N,d}^{abc}(0)
$$

and:

$$
\mathbf{Y}_{N,d}^{abc}
$$

## Step 4: Remove temporary slack constraint

Now solve:

$$
\left(
\mathbf{Y}_{abc}
+
\sum_d
\mathbf{Y}_{N,d}^{abc}
\right)
\mathbf{v}^{abc}
=
\sum_d
\mathbf{i}_{N,d}^{abc}
$$

subject to source symmetry and dynamic equilibrium.

## Step 5: Enforce dynamic consistency

Ensure:

$$
f(x_0,y_0,u_0,t_0)=0
$$

This generalizes the GridLAB-D paper’s Type-1, Type-2, Type-3 method to systems containing both machines and converters.

---

# 22. Numerical Integration

At each time step, solve:

$$
\dot{x}=f(x,y,t)
$$

$$
0=g(x,y,t)
$$

Two common methods are explicit partitioned integration and implicit trapezoidal integration.

---

## 22.1 Explicit Partitioned Method

Given $$x_n$$:

1. Build device Norton equivalents from $$x_n$$.
2. Solve algebraic network:

$$
g(x_n,y_n,t_n)=0
$$

3. Compute derivatives:

$$
\dot{x}_n=f(x_n,y_n,t_n)
$$

4. Advance states:

$$
x_{n+1}
=
x_n
+
\Delta t f(x_n,y_n,t_n)
$$

This is simple but less stable.

---

## 22.2 Trapezoidal Implicit Method

For better stability:

$$
x_{n+1}
=
x_n
+
\frac{\Delta t}{2}
\left[
f(x_n,y_n,t_n)
+
f(x_{n+1},y_{n+1},t_{n+1})
\right]
$$

with algebraic constraint:

$$
0=g(x_{n+1},y_{n+1},t_{n+1})
$$

Define residual:

$$
R_x
=
x_{n+1}
-
x_n
-
\frac{\Delta t}{2}
\left[
f_n
+
f_{n+1}
\right]
$$

$$
R_y
=
g(x_{n+1},y_{n+1},t_{n+1})
$$

The nonlinear system is:

$$
R(x_{n+1},y_{n+1})
=
\begin{bmatrix}
R_x \\
R_y
\end{bmatrix}
=
0
$$

Newton iteration solves:

$$
\begin{bmatrix}
\frac{\partial R_x}{\partial x} &
\frac{\partial R_x}{\partial y} \\
\frac{\partial R_y}{\partial x} &
\frac{\partial R_y}{\partial y}
\end{bmatrix}
\begin{bmatrix}
\Delta x \\
\Delta y
\end{bmatrix}
=
-
\begin{bmatrix}
R_x \\
R_y
\end{bmatrix}
$$

Then:

$$
x_{n+1}^{r+1}
=
x_{n+1}^{r}
+
\Delta x
$$

$$
y_{n+1}^{r+1}
=
y_{n+1}^{r}
+
\Delta y
$$

This is more robust for stiff converter and machine dynamics.

---

# 23. Event Modeling

Events change parameters, topology, or control references.

An event at time $$t_e$$ can be represented as:

$$
u(t)
=
\begin{cases}
u^- , & t<t_e \\
u^+ , & t\geq t_e
\end{cases}
$$

Examples:

## Load trip

$$
P_L(t)
=
\begin{cases}
P_L^0, & t<t_e \\
0, & t\geq t_e
\end{cases}
$$

## Load increase

$$
P_L(t)
=
P_L^0
+
\Delta P_L H(t-t_e)
$$

where $$H$$ is the Heaviside step function.

## Line trip

Before trip:

$$
\mathbf{Y}_{abc}
=
\mathbf{Y}_{abc}^{pre}
$$

After trip:

$$
\mathbf{Y}_{abc}
=
\mathbf{Y}_{abc}^{post}
$$

## Generator trip

Before trip:

$$
\mathbf{i}_{inj,g}
\neq 0
$$

After trip:

$$
\mathbf{i}_{inj,g}=0
$$

or the generator is removed from the network.

---

# 24. Protection and Relay Equations

## 24.1 Under-Frequency Load Shedding

A relay trips if:

$$
\omega(t)<\omega_{th}
$$

for longer than delay $$T_{delay}$$.

Define timer:

$$
\dot{\tau}
=
\begin{cases}
1, & \omega<\omega_{th} \\
0, & \omega\geq\omega_{th}
\end{cases}
$$

Trip condition:

$$
\tau \geq T_{delay}
$$

Then:

$$
P_L \rightarrow P_L(1-\alpha_{shed})
$$

---

## 24.2 Under-Voltage Load Shedding

Similarly:

$$
\dot{\tau}_v
=
\begin{cases}
1, & V<V_{th} \\
0, & V\geq V_{th}
\end{cases}
$$

Trip if:

$$
\tau_v \geq T_{v,delay}
$$

---

## 24.3 Inverter Ride-Through

The inverter remains connected if:

$$
V_{min}(t)
\leq
V(t)
\leq
V_{max}(t)
$$

and:

$$
f_{min}(t)
\leq
f(t)
\leq
f_{max}(t)
$$

Otherwise:

$$
\mathbf{i}_{inj,inv}^{abc}
=
0
$$

or current is limited according to ride-through priority.

---

# 25. Small-Signal Linearization

For stability analysis, linearize the DAE around equilibrium:

$$
x=x_0+\Delta x
$$

$$
y=y_0+\Delta y
$$

At equilibrium:

$$
0=f(x_0,y_0)
$$

$$
0=g(x_0,y_0)
$$

Linearization gives:

$$
\Delta \dot{x}
=
f_x \Delta x
+
f_y \Delta y
$$

$$
0
=
g_x \Delta x
+
g_y \Delta y
$$

If $$g_y$$ is nonsingular:

$$
\Delta y
=
-g_y^{-1}g_x\Delta x
$$

Substitute into the dynamic equation:

$$
\Delta \dot{x}
=
\left(
f_x
-
f_y g_y^{-1}g_x
\right)
\Delta x
$$

Define reduced state matrix:

$$
A_{red}
=
f_x
-
f_y g_y^{-1}g_x
$$

Then:

$$
\Delta \dot{x}
=
A_{red}\Delta x
$$

The eigenvalues of $$A_{red}$$ determine small-signal stability.

If all eigenvalues satisfy:

$$
\operatorname{Re}(\lambda_i)<0
$$

then the equilibrium is locally asymptotically stable.

---

# 26. Final Complete Model

The complete transient hybrid AC/DC distribution-system model is:

$$
\dot{x}
=
f(x,y,u,t)
$$

with:

$$
f
=
\begin{bmatrix}
f_{sg} \\
f_{gov} \\
f_{avr} \\
f_{gfl} \\
f_{gfm} \\
f_{vsc} \\
f_{dcdc} \\
f_{bat} \\
f_{pv} \\
f_{load}
\end{bmatrix}
$$

and:

$$
0
=
g(x,y,u,t)
$$

where:

$$
g
=
\begin{bmatrix}
\mathbf{Y}_{eff}^{abc}(x)\mathbf{v}^{abc}
-
\mathbf{i}_{eff}^{abc}(x,\mathbf{v}^{dc},u,t)
\\
\mathbf{G}_{dc}\mathbf{v}^{dc}
-
\mathbf{i}_{inj}^{dc}(x,\mathbf{v}^{abc},\mathbf{v}^{dc},u,t)
\\
P_{ac,1}+P_{dc,1}+P_{loss,1}+C_{dc,1}V_{dc,1}\dot{V}_{dc,1}
\\
\vdots
\\
P_{ac,N_c}+P_{dc,N_c}+P_{loss,N_c}+C_{dc,N_c}V_{dc,N_c}\dot{V}_{dc,N_c}
\end{bmatrix}
=
0
$$

If DC voltages are treated as dynamic states, the DC equation moves from $$g$$ to $$f$$:

$$
\dot{\mathbf{v}}^{dc}
=
\mathbf{C}_{dc}^{-1}
\left(
\mathbf{i}_{inj}^{dc}
-
\mathbf{G}_{dc}\mathbf{v}^{dc}
\right)
$$

Then the algebraic equation is mainly the AC phasor network:

$$
\mathbf{Y}_{eff}^{abc}(x)
\mathbf{v}^{abc}
=
\mathbf{i}_{eff}^{abc}(x,u,t)
$$

---

# 27. Recommended Mathematical Implementation Flow

For your module, the final simulation algorithm should be:

## Step 1: Static initialization

Solve the hybrid AC/DC PF:

$$
F(z_0)=0
$$

Obtain:

$$
\mathbf{v}_0^{abc},\mathbf{v}_0^{dc},P_0,Q_0
$$

---

## Step 2: Dynamic state initialization

Find $$x_0$$ such that:

$$
f(x_0,y_0,u_0,t_0)=0
$$

---

## Step 3: Device Norton stamping

For each dynamic device:

$$
\mathbf{i}_{inj,d}^{abc}
=
\mathbf{i}_{N,d}^{abc}
-
\mathbf{Y}_{N,d}^{abc}\mathbf{v}_{d}^{abc}
$$

---

## Step 4: Assemble network

$$
\mathbf{Y}_{eff}^{abc}
=
\mathbf{Y}_{abc}
+
\sum_d
\mathbf{C}_d^T
\mathbf{Y}_{N,d}^{abc}
\mathbf{C}_d
$$

$$
\mathbf{i}_{eff}^{abc}
=
\sum_d
\mathbf{C}_d^T
\mathbf{i}_{N,d}^{abc}
$$

---

## Step 5: Solve algebraic network

$$
\mathbf{Y}_{eff}^{abc}
\mathbf{v}^{abc}
=
\mathbf{i}_{eff}^{abc}
$$

and, if algebraic DC is used:

$$
\mathbf{G}_{dc}\mathbf{v}^{dc}
=
\mathbf{i}_{inj}^{dc}
$$

---

## Step 6: Compute powers

For each AC device:

$$
P_d
=
\operatorname{Re}
\left(
(\mathbf{v}_d^{abc})^H
\mathbf{i}_d^{abc}
\right)
$$

$$
Q_d
=
\operatorname{Im}
\left(
(\mathbf{v}_d^{abc})^H
\mathbf{i}_d^{abc}
\right)
$$

For each DC device:

$$
P_d^{dc}
=
V_d^{dc}I_d^{dc}
$$

---

## Step 7: Integrate dynamic states

Use:

$$
x_{n+1}
=
x_n
+
\Delta t f(x_n,y_n,u_n,t_n)
$$

or preferably:

$$
x_{n+1}
=
x_n
+
\frac{\Delta t}{2}
\left[
f_n+f_{n+1}
\right]
$$

---

# 28. Key Conclusion

The mathematical extension you need is not merely the synchronous-machine model from the GridLAB-D paper.

The full enhancement should be:

$$
\text{Static hybrid AC/DC PF}
\rightarrow
\text{three-phase unbalanced dynamic DAE}
$$

with device-level dynamic models:

$$
\text{synchronous machines}
+
\text{grid-following inverters}
+
\text{grid-forming inverters}
+
\text{VSC converters}
+
\text{DC/DC converters}
+
\text{storage}
+
\text{PV}
+
\text{dynamic loads}
$$

The final system is a hybrid dynamic network model:

$$
\boxed{
\dot{x}=f(x,y,u,t)
}
$$

$$
\boxed{
0=g(x,y,u,t)
}
$$

with three-phase AC algebraic constraints:

$$
\boxed{
\mathbf{Y}_{eff}^{abc}(x)\mathbf{v}^{abc}
=
\mathbf{i}_{eff}^{abc}(x,u,t)
}
$$

and DC coupling:

$$
\boxed{
\mathbf{C}_{dc}\dot{\mathbf{v}}^{dc}
=
\mathbf{i}_{inj}^{dc}
-
\mathbf{G}_{dc}\mathbf{v}^{dc}
}
$$

or, if DC is treated algebraically:

$$
\boxed{
\mathbf{G}_{dc}\mathbf{v}^{dc}
=
\mathbf{i}_{inj}^{dc}
}
$$

This gives your module a rigorous mathematical foundation for **three-phase unbalanced hybrid AC/DC transient simulation with synchronous machines and power-electronic devices**.

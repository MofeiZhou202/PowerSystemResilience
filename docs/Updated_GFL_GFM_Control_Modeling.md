> Documentation Sync (2026-07-12)
> Scope: reviewed against current repository structure, CMake presets/options, and registered test targets.
> Status: design/analysis reference; confirm behavior against current implementation before adopting conclusions.
> Source of truth: when text and implementation diverge, treat src/, include/, tests/, and CMake files as authoritative.

# Updated Grid-Following and Grid-Forming Inverter Control Modeling for Three-Phase Dynamic Distribution Simulation

## 1. Purpose

This document updates the inverter dynamic modeling framework for the transient extension of the hybrid AC/DC distribution-system analysis module.

The update is based on the work **“Modeling of Grid-Forming and Grid-Following Inverters for Dynamic Simulation of Large-Scale Distribution Systems”**, which extends GridLAB-D with scalable three-phase electromechanical models for both:

- **Grid-following inverters**, abbreviated as **GFL**,
- **Grid-forming inverters**, abbreviated as **GFM**.

The objective is to replace overly generic inverter models with control structures suitable for large-scale, unbalanced distribution-system dynamic simulation.

The updated modeling philosophy is:

```text
Inverters are not modeled as EMT switching devices.
They are modeled as phasor-domain electromechanical devices.
```

Therefore, the model is intended for:

- feeder-scale dynamic simulation,
- islanded microgrid stability,
- frequency recovery,
- voltage support assessment,
- DER-rich distribution-system transient analysis,
- large-scale RMS or electromechanical simulation.

It is not intended to capture:

- PWM switching,
- harmonics,
- electromagnetic sub-cycle dynamics,
- semiconductor-level behavior,
- detailed filter resonance.

---

# 2. Modeling Level and Assumptions

## 2.1 Phasor-Domain Electromechanical Modeling

All inverter terminal voltages and currents are represented as fundamental-frequency phasors:

$$
\mathbf{V}_{abc}
=
\begin{bmatrix}
V_a \\
V_b \\
V_c
\end{bmatrix}
$$

$$
\mathbf{I}_{abc}
=
\begin{bmatrix}
I_a \\
I_b \\
I_c
\end{bmatrix}
$$

The network is solved in full three-phase coordinates:

$$
\mathbf{Y}_{abc}\mathbf{V}_{abc}
=
\mathbf{I}_{inj,abc}
$$

This allows the model to represent:

- unbalanced lines,
- single-phase laterals,
- unequal phase loading,
- phase-specific voltage deviations,
- phase-specific current injections.

---

## 2.2 Constant DC Bus Assumption

In the referenced GridLAB-D inverter modeling work, the DC bus is assumed constant for the inverter dynamic model:

$$
V_{dc} = V_{dc}^{ref}
$$

This assumption is reasonable for:

- battery energy storage inverters with strong DC-side regulation,
- gas-driven inverters,
- inverter sources with sufficient DC-side surge capability,
- studies focused primarily on AC-side frequency and voltage dynamics.

However, for the hybrid AC/DC module, this assumption should be configurable.

Recommended implementation:

```cpp
enum class DCLinkMode {
    ConstantDCVoltage,
    DynamicDCVoltage
};
```

If dynamic DC voltage is enabled, use:

$$
C_{dc}V_{dc}\dot{V}_{dc}
=
P_{dc,in}
-
P_{ac,out}
-
P_{loss}
$$

or, with bus-injection sign convention:

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

---

## 2.3 Algebraic Filter Representation

The inverter output filter is represented algebraically using a coupling reactance:

$$
X_L
$$

A typical value used in the reference work is:

$$
X_L = 0.1 \ \text{pu}
$$

The filter is not modeled as a differential equation. Instead, it is represented as an algebraic impedance between the internal inverter voltage and the terminal bus voltage.

For a balanced internal voltage source:

$$
\mathbf{E}_{abc}
=
\begin{bmatrix}
E_a \\
E_b \\
E_c
\end{bmatrix}
$$

the terminal current is:

$$
\mathbf{I}_{abc}
=
\mathbf{Y}_L
\left(
\mathbf{E}_{abc}
-
\mathbf{V}_{abc}
\right)
$$

where:

$$
\mathbf{Y}_L
=
\frac{1}{jX_L}
\mathbf{I}_3
$$

and $$\mathbf{I}_3$$ is the $$3 \times 3$$ identity matrix.

---

## 2.4 Norton Equivalent Interface

To interface with a current-injection network solver, the inverter voltage-source representation is converted to a Norton equivalent.

For each phase:

$$
I_I \angle \phi_I
=
\frac{
E \angle \delta
}{
jX_L
}
$$

The Norton admittance is:

$$
Y_L
=
\frac{1}{jX_L}
$$

In vector form:

$$
\mathbf{I}_{inj,abc}
=
\mathbf{I}_{N,abc}
-
\mathbf{Y}_L\mathbf{V}_{abc}
$$

where:

$$
\mathbf{I}_{N,abc}
=
\mathbf{Y}_L\mathbf{E}_{abc}
$$

Therefore:

$$
\mathbf{I}_{inj,abc}
=
\mathbf{Y}_L\mathbf{E}_{abc}
-
\mathbf{Y}_L\mathbf{V}_{abc}
$$

This form is directly compatible with the dynamic network equation:

$$
\left(
\mathbf{Y}_{abc}
+
\sum_d \mathbf{Y}_{N,d}
\right)
\mathbf{V}_{abc}
=
\sum_d \mathbf{I}_{N,d}
$$

---

# 3. Updated Grid-Following Inverter Model

## 3.1 Physical Interpretation

A grid-following inverter behaves primarily as a controlled current source.

It does not create the grid voltage or frequency. Instead, it synchronizes to an existing voltage waveform using a phase-locked loop.

Therefore, a GFL inverter requires an external voltage/frequency reference, such as:

- substation source,
- synchronous generator,
- grid-forming inverter,
- another strong voltage source.

In islanded systems with insufficient grid-forming capacity, high GFL penetration can degrade stability because GFL inverters attempt to maintain their own power references instead of naturally sharing disturbances.

---

## 3.2 GFL Control Structure

The GFL control model contains:

1. **Phase-locked loop**, abbreviated as PLL.
2. **Active/reactive power controller** or direct current-reference generator.
3. **Current control loop**.
4. **Positive-sequence current injection block**.
5. **Current-limiting logic**.
6. Optional voltage-support and frequency-watt functions.

The conceptual structure is:

```text
Measured Vabc
    ↓
Positive-sequence extraction
    ↓
PLL
    ↓
dq transformation
    ↓
P/Q reference to id/iq reference
    ↓
Current controller
    ↓
Current limiter
    ↓
Positive-sequence current injection
    ↓
Three-phase network solver
```

---

## 3.3 Positive-Sequence Voltage Extraction

Because the distribution feeder is unbalanced, the terminal voltages are:

$$
\mathbf{V}_{abc}
=
\begin{bmatrix}
V_a \\
V_b \\
V_c
\end{bmatrix}
$$

The positive-sequence voltage is:

$$
V^1
=
\frac{1}{3}
\left(
V_a
+
aV_b
+
a^2V_c
\right)
$$

where:

$$
a
=
e^{j\frac{2\pi}{3}}
$$

The GFL inverter controller uses this positive-sequence voltage for synchronization.

The internal controller therefore sees:

$$
V_{pos}
=
V^1
$$

while the network still solves the full unbalanced three-phase voltages.

---

## 3.4 PLL Model

The PLL estimates the grid phase angle:

$$
\delta_{PLL}
$$

and angular frequency:

$$
\omega_{PLL}
$$

The positive-sequence terminal voltage is transformed into the PLL reference frame:

$$
\begin{bmatrix}
v_d^{PLL} \\
v_q^{PLL}
\end{bmatrix}
=
T_{dq}(\delta_{PLL})V^1
$$

The PLL attempts to drive:

$$
v_q^{PLL}
\rightarrow 0
$$

Define the PLL integrator state:

$$
\xi_{PLL}
$$

The PI PLL is:

$$
\dot{\xi}_{PLL}
=
v_q^{PLL}
$$

$$
\omega_{PLL}
=
\omega_0
+
k_{pPLL}v_q^{PLL}
+
k_{iPLL}\xi_{PLL}
$$

$$
\dot{\delta}_{PLL}
=
\omega_{PLL}
$$

Recommended initial parameters from the reference work:

$$
k_{pPLL} = 0.01
$$

$$
k_{iPLL} = 1.0
$$

---

## 3.5 Power Calculation in the PLL Frame

The measured dq voltage and current are:

$$
\mathbf{v}_{dq}
=
\begin{bmatrix}
v_d \\
v_q
\end{bmatrix}
$$

$$
\mathbf{i}_{dq}
=
\begin{bmatrix}
i_d \\
i_q
\end{bmatrix}
$$

The active and reactive power are:

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

If the PLL is well aligned:

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

---

## 3.6 Current Reference Generation

Given active and reactive power references:

$$
P^{ref}
$$

$$
Q^{ref}
$$

the corresponding current references are obtained from:

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

Therefore:

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

To avoid numerical issues during low-voltage events, use:

$$
v_d^2+v_q^2
\geq
V_{min,current}^2
$$

In implementation:

```cpp
double denom = std::max(vd * vd + vq * vq, VminCurrent * VminCurrent);
id_ref = ( vd * Pref + vq * Qref) / denom;
iq_ref = ( vq * Pref - vd * Qref) / denom;
```

---

## 3.7 Current Control Loop

The current loop is represented as a first-order electromechanical approximation.

A simple first-order current-tracking model is:

$$
T_c\dot{i}_d
=
-i_d
+
i_d^{cmd}
$$

$$
T_c\dot{i}_q
=
-i_q
+
i_q^{cmd}
$$

where:

$$
i_d^{cmd}
$$

and:

$$
i_q^{cmd}
$$

are current-limited references.

Equivalently:

$$
\dot{i}_d
=
\frac{
i_d^{cmd}-i_d
}{
T_c
}
$$

$$
\dot{i}_q
=
\frac{
i_q^{cmd}-i_q
}{
T_c
}
$$

The reference paper reports PI current-loop parameters:

$$
k_{pc}=0.1
$$

$$
k_{ic}=10.0
$$

For large-scale electromechanical simulation, the PI loop can be represented either explicitly or approximated by a first-order equivalent.

If explicit PI states are desired:

$$
e_d
=
i_d^{ref}
-
i_d
$$

$$
e_q
=
i_q^{ref}
-
i_q
$$

$$
\dot{\xi}_{id}
=
e_d
$$

$$
\dot{\xi}_{iq}
=
e_q
$$

$$
u_d
=
k_{pc}e_d
+
k_{ic}\xi_{id}
$$

$$
u_q
=
k_{pc}e_q
+
k_{ic}\xi_{iq}
$$

For feeder-scale dynamic simulation, the first-order approximation is usually preferred for robustness.

---

## 3.8 Current Limiting

The inverter current magnitude must satisfy:

$$
i_d^2+i_q^2
\leq
I_{max}^2
$$

If:

$$
\sqrt{
(i_d^{ref})^2
+
(i_q^{ref})^2
}
\leq
I_{max}
$$

then:

$$
i_d^{cmd}
=
i_d^{ref}
$$

$$
i_q^{cmd}
=
i_q^{ref}
$$

Otherwise, proportional limiting gives:

$$
\lambda
=
\frac{
I_{max}
}{
\sqrt{
(i_d^{ref})^2
+
(i_q^{ref})^2
}
}
$$

$$
i_d^{cmd}
=
\lambda i_d^{ref}
$$

$$
i_q^{cmd}
=
\lambda i_q^{ref}
$$

For fault ride-through or voltage-support operation, reactive current priority can be used:

$$
i_q^{cmd}
=
\operatorname{sat}
\left(
i_q^{ref},
-I_{max},
I_{max}
\right)
$$

$$
i_d^{cmd}
=
\operatorname{sgn}(i_d^{ref})
\sqrt{
I_{max}^2
-
(i_q^{cmd})^2
}
$$

---

## 3.9 Positive-Sequence Current Injection

A key modeling decision in the referenced work is:

```text
GFL inverters inject positive-sequence current only.
```

This means that even though the terminal voltage is unbalanced, the inverter current command is internally balanced.

The balanced positive-sequence current phasors are:

$$
\mathbf{I}_{abc}^{GFL}
=
\begin{bmatrix}
I_a \\
I_b \\
I_c
\end{bmatrix}
=
I
\begin{bmatrix}
e^{j\delta_{PLL}} \\
e^{j(\delta_{PLL}-\frac{2\pi}{3})} \\
e^{j(\delta_{PLL}+\frac{2\pi}{3})}
\end{bmatrix}
$$

More generally, from dq currents:

$$
\mathbf{I}_{abc}^{GFL}
=
T_{abc}(\delta_{PLL})
\begin{bmatrix}
i_d \\
i_q \\
0
\end{bmatrix}
$$

The current source is stamped into the network as:

$$
\mathbf{I}_{inj,abc}^{GFL}
=
\mathbf{I}_{abc}^{GFL}
$$

Optionally, add a small numerical stabilizing admittance:

$$
\mathbf{I}_{inj,abc}^{GFL}
=
\mathbf{I}_{abc}^{GFL}
-
\mathbf{Y}_{stab}\mathbf{V}_{abc}
$$

---

## 3.10 GFL State Vector

A recommended GFL state vector is:

$$
x_{GFL}
=
\begin{bmatrix}
\delta_{PLL} \\
\xi_{PLL} \\
i_d \\
i_q
\end{bmatrix}
$$

If explicit PI current control is included:

$$
x_{GFL}
=
\begin{bmatrix}
\delta_{PLL} \\
\xi_{PLL} \\
i_d \\
i_q \\
\xi_{id} \\
\xi_{iq}
\end{bmatrix}
$$

If DC-link dynamics are included:

$$
x_{GFL}
=
\begin{bmatrix}
\delta_{PLL} \\
\xi_{PLL} \\
i_d \\
i_q \\
V_{dc}
\end{bmatrix}
$$

---

## 3.11 GFL Dynamic Equations Summary

The complete reduced GFL model is:

$$
\dot{\xi}_{PLL}
=
v_q^{PLL}
$$

$$
\omega_{PLL}
=
\omega_0
+
k_{pPLL}v_q^{PLL}
+
k_{iPLL}\xi_{PLL}
$$

$$
\dot{\delta}_{PLL}
=
\omega_{PLL}
$$

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

$$
i_d^{cmd},i_q^{cmd}
=
\operatorname{CurrentLimit}
\left(
i_d^{ref},i_q^{ref},I_{max}
\right)
$$

$$
\dot{i}_d
=
\frac{
i_d^{cmd}-i_d
}{
T_c
}
$$

$$
\dot{i}_q
=
\frac{
i_q^{cmd}-i_q
}{
T_c
}
$$

$$
\mathbf{I}_{inj,abc}^{GFL}
=
T_{abc}(\delta_{PLL})
\begin{bmatrix}
i_d \\
i_q \\
0
\end{bmatrix}
$$

---

# 4. Updated Grid-Forming Inverter Model

## 4.1 Physical Interpretation

A grid-forming inverter behaves primarily as a controlled voltage source.

It can establish:

- voltage magnitude,
- voltage phase angle,
- frequency reference.

Therefore, it can support islanded microgrid operation.

Compared with GFL inverters, GFM inverters can:

- provide immediate active-power support after generation loss,
- improve voltage profiles,
- share load changes through droop,
- reduce frequency nadir,
- reduce or avoid under-frequency load shedding,
- support inverter-dominated islanded feeders.

---

## 4.2 GFM Control Structure

The updated GFM model is based on CERTS-style droop control.

The main blocks are:

1. **Power measurement filter**.
2. **P-f droop control**.
3. **Q-V droop control**.
4. **Voltage controller**.
5. **Overload mitigation controller**.
6. **Balanced internal voltage source generation**.
7. **Algebraic filter reactance and Norton equivalent**.

Conceptual structure:

```text
Measured Vabc, Iabc
    ↓
Three-phase P and Q calculation
    ↓
Low-pass power filters
    ↓
P-f droop and Q-V droop
    ↓
Overload mitigation
    ↓
Internal balanced voltage Eabc
    ↓
Coupling reactance XL
    ↓
Norton equivalent
    ↓
Three-phase network solver
```

---

## 4.3 Three-Phase Power Measurement

The complex power at the inverter terminal is:

$$
S
=
P+jQ
=
V_a I_a^*
+
V_b I_b^*
+
V_c I_c^*
$$

Therefore:

$$
P
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
Q
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

These total three-phase powers are used by the GFM droop controller.

---

## 4.4 Power Measurement Filter

To avoid algebraic sensitivity and to represent controller measurement delay, filtered powers are used.

Let:

$$
P_f
$$

and:

$$
Q_f
$$

be filtered active and reactive powers.

The first-order low-pass filters are:

$$
T\dot{P}_f
=
P
-
P_f
$$

$$
T\dot{Q}_f
=
Q
-
Q_f
$$

or:

$$
\dot{P}_f
=
\frac{
P-P_f
}{
T
}
$$

$$
\dot{Q}_f
=
\frac{
Q-Q_f
}{
T
}
$$

Recommended value from the reference work:

$$
T = 0.01 \ \text{s}
$$

---

## 4.5 P-f Droop Control

The P-f droop controller adjusts inverter frequency according to active power.

A standard droop relation is:

$$
\omega^{cmd}
=
\omega_0
-
m_p
\left(
P_f
-
P^{ref}
\right)
$$

The internal voltage angle evolves as:

$$
\dot{\delta}
=
\omega^{cmd}
$$

Equivalently, if frequency is represented in Hz:

$$
f^{cmd}
=
f_0
-
m_p
\left(
P_f
-
P^{ref}
\right)
$$

$$
\dot{\delta}
=
2\pi f^{cmd}
$$

Recommended droop parameter from the reference work:

$$
m_p = 0.01
$$

The exact unit of $$m_p$$ must be consistent with the selected per-unit or physical-unit implementation.

---

## 4.6 Q-V Droop Control

The Q-V droop controller adjusts voltage magnitude according to reactive power.

A standard relation is:

$$
E^{droop}
=
E_0
-
m_q
\left(
Q_f
-
Q^{ref}
\right)
$$

Recommended droop parameter from the reference work:

$$
m_q = 0.05
$$

This control helps reduce circulating reactive power and improves voltage sharing among multiple grid-forming inverters.

---

## 4.7 Voltage Control Loop

The reference work includes voltage-loop PI parameters:

$$
k_{pv} = 0.1
$$

$$
k_{iv} = 10.0
$$

A practical feeder-scale voltage controller can be written as:

$$
e_v
=
E^{droop}
-
V_t
$$

$$
\dot{\xi}_v
=
e_v
$$

$$
E^{cmd}
=
E^{droop}
+
k_{pv}e_v
+
k_{iv}\xi_v
$$

However, for large-scale electromechanical simulation, a simplified first-order voltage loop is often more robust:

$$
T_v\dot{E}
=
-E
+
E^{cmd}
$$

or:

$$
\dot{E}
=
\frac{
E^{cmd}-E
}{
T_v
}
$$

where $$E$$ is the internal voltage magnitude.

If a direct droop voltage model is desired, use:

$$
E
=
E^{droop}
$$

without introducing an additional voltage-loop state.

---

## 4.8 Balanced Internal Voltage Constraint

The GFM inverter maintains a balanced internal voltage source even when terminal voltages are unbalanced.

Therefore:

$$
E_a = E
$$

$$
E_b = E
$$

$$
E_c = E
$$

with phase separation of $$120^\circ$$:

$$
\mathbf{E}_{abc}
=
E
\begin{bmatrix}
e^{j\delta} \\
e^{j(\delta-\frac{2\pi}{3})} \\
e^{j(\delta+\frac{2\pi}{3})}
\end{bmatrix}
$$

This is important in unbalanced distribution systems:

```text
The terminal voltage can be unbalanced.
The internal GFM voltage source remains balanced.
```

This behavior helps mitigate feeder voltage imbalance.

---

## 4.9 Coupling Reactance and Norton Equivalent

The internal balanced voltage source is connected to the terminal bus through coupling reactance:

$$
X_L
$$

The phase admittance is:

$$
Y_L
=
\frac{1}{jX_L}
$$

In matrix form:

$$
\mathbf{Y}_L
=
Y_L
\mathbf{I}_3
$$

The terminal current injection is:

$$
\mathbf{I}_{inj,abc}^{GFM}
=
\mathbf{Y}_L
\left(
\mathbf{E}_{abc}
-
\mathbf{V}_{abc}
\right)
$$

Therefore, the Norton current source is:

$$
\mathbf{I}_{N,abc}^{GFM}
=
\mathbf{Y}_L\mathbf{E}_{abc}
$$

and the Norton admittance is:

$$
\mathbf{Y}_{N}^{GFM}
=
\mathbf{Y}_L
$$

The network stamp is:

$$
\mathbf{I}_{inj,abc}^{GFM}
=
\mathbf{I}_{N,abc}^{GFM}
-
\mathbf{Y}_{N}^{GFM}\mathbf{V}_{abc}
$$

In the global network equation:

$$
\left(
\mathbf{Y}_{abc}
+
\mathbf{Y}_{N}^{GFM}
\right)
\mathbf{V}_{abc}
=
\mathbf{I}_{N,abc}^{GFM}
+
\mathbf{I}_{other}
$$

---

## 4.10 Overload Mitigation Control

A major feature of the referenced GFM model is overload mitigation.

The inverter has active power limits:

$$
P_{min}
\leq
P
\leq
P_{max}
$$

If the inverter is overloaded:

$$
P_f > P_{max}
$$

then the controller intentionally reduces the frequency command.

This can trigger under-frequency load shedding in the external system, allowing the islanded microgrid to stabilize.

Define overload error:

$$
e_{Pmax}
=
P_f
-
P_{max}
$$

The overload error is active only when:

$$
e_{Pmax} > 0
$$

Use:

$$
e_{ol}
=
\max
\left(
0,
P_f-P_{max}
\right)
$$

The overload PI state is:

$$
\dot{\xi}_{ol}
=
e_{ol}
$$

The overload frequency correction is:

$$
\Delta \omega_{ol}
=
k_{ppmax}e_{ol}
+
k_{ipmax}\xi_{ol}
$$

The modified frequency command is:

$$
\omega^{cmd}
=
\omega_0
-
m_p
\left(
P_f-P^{ref}
\right)
-
\Delta \omega_{ol}
$$

Recommended overload-controller parameters from the reference work:

$$
k_{ppmax}=0.1
$$

$$
k_{ipmax}=10.0
$$

A lower active-power limit can be handled similarly:

$$
e_{Pmin}
=
\max
\left(
0,
P_{min}-P_f
\right)
$$

and can increase the frequency command if needed:

$$
\omega^{cmd}
=
\omega_0
-
m_p
\left(
P_f-P^{ref}
\right)
-
\Delta \omega_{Pmax}
+
\Delta \omega_{Pmin}
$$

---

## 4.11 GFM Current and Apparent Power Limits

Even though the GFM inverter is modeled as a voltage source, it must still respect converter limits.

The output current is:

$$
\mathbf{I}_{abc}^{GFM}
=
\mathbf{Y}_L
\left(
\mathbf{E}_{abc}
-
\mathbf{V}_{abc}
\right)
$$

The current magnitude can be evaluated as:

$$
I_{rms}
=
\sqrt{
\frac{
|I_a|^2+|I_b|^2+|I_c|^2
}{
3
}
}
$$

The current limit is:

$$
I_{rms}
\leq
I_{max}
$$

The apparent power is:

$$
S
=
\sqrt{
P^2+Q^2
}
$$

and must satisfy:

$$
S
\leq
S_{rated}
$$

Recommended practical implementation:

1. First use droop and overload mitigation to reduce active power stress.
2. If current still exceeds the limit, reduce internal voltage magnitude $$E$$ or increase virtual impedance.
3. If current remains excessive, trip or block the inverter according to protection settings.

A simple current-limited voltage scaling is:

$$
\gamma
=
\min
\left(
1,
\frac{
I_{max}
}{
I_{rms}
}
\right)
$$

$$
E^{limited}
=
V_t
+
\gamma
\left(
E^{cmd}
-
V_t
\right)
$$

This approximate limiter reduces the voltage-source strength while maintaining the voltage-source behavior.

---

## 4.12 GFM State Vector

A recommended GFM state vector is:

$$
x_{GFM}
=
\begin{bmatrix}
\delta \\
P_f \\
Q_f \\
E \\
\xi_v \\
\xi_{ol}
\end{bmatrix}
$$

A simplified state vector is:

$$
x_{GFM}
=
\begin{bmatrix}
\delta \\
P_f \\
Q_f
\end{bmatrix}
$$

if direct algebraic voltage droop is used:

$$
E = E^{droop}
$$

and no overload PI integrator is included.

A more detailed state vector is:

$$
x_{GFM}
=
\begin{bmatrix}
\delta \\
\omega \\
P_f \\
Q_f \\
E \\
\xi_v \\
\xi_{ol}
\end{bmatrix}
$$

if frequency is explicitly filtered or represented as a state.

---

## 4.13 GFM Dynamic Equations Summary

The complete updated GFM model is:

$$
P
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
Q
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

$$
\dot{P}_f
=
\frac{
P-P_f
}{
T
}
$$

$$
\dot{Q}_f
=
\frac{
Q-Q_f
}{
T
}
$$

$$
e_{ol}
=
\max
\left(
0,
P_f-P_{max}
\right)
$$

$$
\dot{\xi}_{ol}
=
e_{ol}
$$

$$
\Delta \omega_{ol}
=
k_{ppmax}e_{ol}
+
k_{ipmax}\xi_{ol}
$$

$$
\omega^{cmd}
=
\omega_0
-
m_p
\left(
P_f-P^{ref}
\right)
-
\Delta \omega_{ol}
$$

$$
\dot{\delta}
=
\omega^{cmd}
$$

$$
E^{droop}
=
E_0
-
m_q
\left(
Q_f-Q^{ref}
\right)
$$

If using direct voltage droop:

$$
E = E^{droop}
$$

If using first-order voltage loop:

$$
\dot{E}
=
\frac{
E^{cmd}-E
}{
T_v
}
$$

with:

$$
E^{cmd}
=
E^{droop}
+
k_{pv}
\left(
E^{droop}-V_t
\right)
+
k_{iv}\xi_v
$$

$$
\dot{\xi}_v
=
E^{droop}
-
V_t
$$

The internal voltage source is:

$$
\mathbf{E}_{abc}
=
E
\begin{bmatrix}
e^{j\delta} \\
e^{j(\delta-\frac{2\pi}{3})} \\
e^{j(\delta+\frac{2\pi}{3})}
\end{bmatrix}
$$

The Norton current source is:

$$
\mathbf{I}_{N,abc}^{GFM}
=
\mathbf{Y}_L\mathbf{E}_{abc}
$$

The network injection is:

$$
\mathbf{I}_{inj,abc}^{GFM}
=
\mathbf{I}_{N,abc}^{GFM}
-
\mathbf{Y}_L\mathbf{V}_{abc}
$$

---

# 5. Comparison Between Updated GFL and GFM Models

| Feature | Grid-Following Inverter | Grid-Forming Inverter |
|---|---|---|
| Physical behavior | Current source | Voltage source |
| Needs external grid reference | Yes | No |
| Synchronization | PLL | Internal angle/frequency |
| Main control | P/Q current control | P-f and Q-V droop |
| Islanded operation | Not sufficient alone | Suitable |
| Fault/load-step response | Returns to power setpoint | Shares load dynamically |
| Voltage support | Limited unless explicitly controlled | Natural through voltage source and Q-V droop |
| Frequency support | Limited unless frequency-watt added | Natural through P-f droop |
| Unbalanced network handling | Positive-sequence current injection | Balanced internal voltage source |
| Norton representation | Current source, optional stabilizing admittance | Voltage source behind reactance |
| Typical time step | Smaller, about 2 ms in reference study | Larger, about 5 ms in reference study |
| Large-scale stability benefit | Lower | Higher |

---

# 6. Updated C++ Implementation Recommendation

## 6.1 Base Inverter Interface

```cpp
class DynamicInverter : public DynamicDevice {
public:
    virtual InverterMode mode() const = 0;

    virtual void computePower(
        const NetworkState& y,
        double& P,
        double& Q
    ) const = 0;

    virtual void applyCurrentLimit() = 0;

    virtual void stamp(
        double t,
        const DynamicState& x,
        const NetworkState& y,
        DynamicStamp& stamp
    ) const override = 0;
};
```

Recommended enum:

```cpp
enum class InverterMode {
    GridFollowing,
    GridForming
};
```

---

## 6.2 GFL Class

```cpp
class GridFollowingInverter : public DynamicInverter {
public:
    int acBus;
    int dcBus;

    double Pref;
    double Qref;

    double kpPLL = 0.01;
    double kiPLL = 1.0;

    double kpc = 0.1;
    double kic = 10.0;

    double Tc;
    double Imax;

    StateIndexRange range;

    void initializeFromPowerFlow(
        const PowerFlowResult& pf,
        DynamicState& x,
        NetworkState& y
    ) override;

    void computeDerivatives(
        double t,
        const DynamicState& x,
        const NetworkState& y,
        Eigen::Ref<Eigen::VectorXd> dxdt
    ) const override;

    void stamp(
        double t,
        const DynamicState& x,
        const NetworkState& y,
        DynamicStamp& stamp
    ) const override;
};
```

Suggested state indices:

```cpp
enum GFLState {
    DELTA_PLL = 0,
    XI_PLL = 1,
    ID = 2,
    IQ = 3,
    XI_ID = 4,
    XI_IQ = 5
};
```

---

## 6.3 GFM Class

```cpp
class GridFormingInverter : public DynamicInverter {
public:
    int acBus;
    int dcBus;

    double Pref;
    double Qref;

    double E0;
    double omega0;

    double mp = 0.01;
    double mq = 0.05;

    double kpv = 0.1;
    double kiv = 10.0;

    double kppmax = 0.1;
    double kipmax = 10.0;

    double T = 0.01;
    double XL = 0.1;

    double Pmax;
    double Pmin;
    double Imax;
    double Srated;

    StateIndexRange range;

    void initializeFromPowerFlow(
        const PowerFlowResult& pf,
        DynamicState& x,
        NetworkState& y
    ) override;

    void computeDerivatives(
        double t,
        const DynamicState& x,
        const NetworkState& y,
        Eigen::Ref<Eigen::VectorXd> dxdt
    ) const override;

    void stamp(
        double t,
        const DynamicState& x,
        const NetworkState& y,
        DynamicStamp& stamp
    ) const override;
};
```

Suggested state indices:

```cpp
enum GFMState {
    DELTA = 0,
    PF = 1,
    QF = 2,
    E = 3,
    XI_V = 4,
    XI_OL = 5
};
```

---

# 7. Recommended Solver Settings

The reference work shows that GFL models require smaller time steps because of the fast PLL and current-control dynamics.

Recommended default settings:

| Model | Recommended time step | Notes |
|---|---:|---|
| GFL inverter | 1 ms to 2 ms | Needed for fast PLL/current loop |
| GFM inverter | 2 ms to 5 ms | More stable due to voltage-source behavior |
| Mixed SG + GFM | 2 ms to 5 ms | Good for islanded microgrid studies |
| Mixed SG + GFL | 1 ms to 2 ms | Watch frequency and current-loop stiffness |
| Large feeder, planning-level dynamic | 5 ms if mostly GFM | Validate stability |
| Converter-rich dynamic simulation | adaptive implicit solver preferred | Use IDA or backward Euler |

Recommended solver stack:

```text
Prototype:
    Partitioned RK4 or Heun
    Eigen SparseLU

Robust native solver:
    Backward Euler or trapezoidal Newton
    SuiteSparse KLU

Production solver:
    SUNDIALS IDA
    KLU or PARDISO
```

---

# 8. Recommended Parameter Defaults

## 8.1 GFL Parameters

| Parameter | Default | Description |
|---|---:|---|
| $$k_{pPLL}$$ | 0.01 | PLL proportional gain |
| $$k_{iPLL}$$ | 1.0 | PLL integral gain |
| $$k_{pc}$$ | 0.1 | Current-loop proportional gain |
| $$k_{ic}$$ | 10.0 | Current-loop integral gain |
| $$X_L$$ | 0.1 pu | Coupling reactance if voltage-source interface is used |
| $$I_{max}$$ | device-specific | Maximum inverter current |
| $$T_c$$ | derived from current loop | First-order current loop time constant |

---

## 8.2 GFM Parameters

| Parameter | Default | Description |
|---|---:|---|
| $$m_p$$ | 0.01 | P-f droop gain |
| $$m_q$$ | 0.05 | Q-V droop gain |
| $$k_{pv}$$ | 0.1 | Voltage-loop proportional gain |
| $$k_{iv}$$ | 10.0 | Voltage-loop integral gain |
| $$k_{ppmax}$$ | 0.1 | Overload PI proportional gain |
| $$k_{ipmax}$$ | 10.0 | Overload PI integral gain |
| $$T$$ | 0.01 s | Power measurement low-pass filter time constant |
| $$X_L$$ | 0.1 pu | Coupling reactance |
| $$P_{max}$$ | device-specific | Maximum active power |
| $$P_{min}$$ | device-specific | Minimum active power |
| $$I_{max}$$ | device-specific | Maximum current |
| $$S_{rated}$$ | device-specific | Apparent power rating |

---

# 9. Integration With the Hybrid AC/DC Module

## 9.1 AC-Side Integration

Both GFL and GFM devices are connected to the three-phase AC network through current injections.

The network equation is:

$$
\mathbf{Y}_{eff}^{abc}
\mathbf{V}_{abc}
=
\mathbf{I}_{eff}^{abc}
$$

For GFL:

$$
\mathbf{I}_{eff}^{abc}
\leftarrow
\mathbf{I}_{eff}^{abc}
+
\mathbf{I}_{abc}^{GFL}
$$

For GFM:

$$
\mathbf{Y}_{eff}^{abc}
\leftarrow
\mathbf{Y}_{eff}^{abc}
+
\mathbf{Y}_L
$$

$$
\mathbf{I}_{eff}^{abc}
\leftarrow
\mathbf{I}_{eff}^{abc}
+
\mathbf{Y}_L\mathbf{E}_{abc}
$$

---

## 9.2 DC-Side Integration

If constant DC voltage is used, the DC side is not dynamically simulated inside the inverter model.

If dynamic DC coupling is enabled, then:

$$
P_{ac}
+
P_{dc}
+
P_{loss}
+
C_{dc}V_{dc}\dot{V}_{dc}
=
0
$$

The corresponding DC current injection is:

$$
I_{dc}
=
\frac{
P_{dc}
}{
V_{dc}
}
$$

The DC network equation is:

$$
\mathbf{C}_{dc}\dot{\mathbf{V}}_{dc}
=
\mathbf{I}_{inj,dc}
-
\mathbf{G}_{dc}\mathbf{V}_{dc}
$$

or algebraically:

$$
\mathbf{G}_{dc}\mathbf{V}_{dc}
=
\mathbf{I}_{inj,dc}
$$

---

# 10. Recommended Validation Cases

## 10.1 GFL Validation

Use a single 100 kW inverter connected to an infinite bus.

Apply a step change:

$$
P^{ref}: 0.5 \ \text{pu} \rightarrow 1.0 \ \text{pu}
$$

Expected behavior:

- PLL remains synchronized.
- Current loop tracks new active current reference.
- Active power rises to new reference.
- Differences from EMT appear mainly in fast filter/line transients.

---

## 10.2 GFM Validation

Use a CERTS-style microgrid.

Events:

- trip one energy storage source,
- observe active power sharing,
- observe frequency trajectory,
- activate overload mitigation if needed,
- confirm UFLS triggering when generation is insufficient.

Expected behavior:

- GFM sources autonomously increase output after loss of generation,
- frequency nadir is improved relative to GFL-only cases,
- voltage profile is improved due to voltage-source behavior,
- overload mitigation can intentionally reduce frequency to trigger UFLS.

---

## 10.3 Large Feeder Validation

Use a large unbalanced islanded feeder with:

- synchronous generators,
- GFL inverters,
- GFM inverters,
- under-frequency load shedding,
- unbalanced primary and secondary circuits.

Compare cases:

1. All inverters GFL.
2. All inverters GFM.
3. Mixed GFL/GFM.
4. Sequential generator loss.
5. Load increase.
6. UFLS action.

Expected system-level result:

```text
GFM penetration should improve frequency nadir, voltage profile,
dynamic power sharing, and islanded survivability.
```

---

# 11. Final Recommended Modeling Choice

For the hybrid AC/DC transient module, the inverter library should include both models:

```text
GridFollowingInverter:
    PLL-based positive-sequence current source

GridFormingInverter:
    CERTS droop-controlled balanced voltage source behind reactance
```

The GFL model is appropriate for:

- PV operating in normal grid-connected mode,
- PQ-controlled battery inverters,
- current-controlled VSCs,
- DERs that rely on a strong grid reference.

The GFM model is appropriate for:

- islanded microgrids,
- black-start-capable battery systems,
- grid-supporting VSCs,
- inverter-dominated feeders,
- resilience studies.

The most important update from the referenced work is:

```text
GFL inverters should be modeled as positive-sequence current sources.
GFM inverters should be modeled as balanced internal voltage sources
behind algebraic coupling reactance, with CERTS-style P-f and Q-V droop.
```

This distinction is essential for accurately studying dynamic stability in large-scale unbalanced distribution systems.

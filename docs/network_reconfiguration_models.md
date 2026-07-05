> Documentation Sync (2026-07-05)
> Scope: reviewed against current repository structure, CMake presets/options, and registered test targets.
> Status: implementation-backed reference.
> Source of truth: when text and implementation diverge, treat src/, include/, tests/, and CMake files as authoritative.

# Optimal Network Reconfiguration — Mathematical Models (Canonical Space)

This document is the engineering/math reference for the hybrid AC/DC optimal
network reconfiguration (ONR). It supersedes the prose in
`docs/technical_notebook/sections/07_network_reconfiguration.tex`, restated as
configurable constraint groups, objectives, and an optional power-flow layer so
they can be toggled from the GUI. CBs, switches, AC/DC lines, and DC/DC
converters are optimized over the **canonical** model space, then projected back
to device operations.

> **Companion / enhancement:** the reliability assessment math reference and
> rigor audit — [`reliability_assessment_models.md`](reliability_assessment_models.md) —
> documents how this ONR LinDistFlow model is reused as the per-stage restoration
> kernel of the FMEA repair search and the three-stage reliability MILP, and
> classifies every reliability formula as rigorous or heuristic.

Implementation: `src/network_reconfiguration/topology_reconfiguration.cpp`
(`run_topology_reconfiguration`) and `topology_analysis.cpp`
(`solve_optimal_reconfiguration`).

## 1. Canonical projection and the hybrid edge set

Solve over $\widehat{\mathcal S}=\Pi_{\text{canon}}(\mathcal S)$ with
`project_to_canonical_models(sys, strip_dead=false)` (dead sections kept so open
ties remain candidates). Edges:

$$\mathcal E_{\text{hyb}}=\mathcal E_{ac}^{\text{canon}}\cup\mathcal E_{dc}^{\text{canon}}\cup\mathcal E_{vsc}^{\text{canon}}.$$

Devices map to canonical edges via `BranchExpandMap`: each `Switch`/`CircuitBreaker` → one `ACBranch` with endpoint buses and closed state. The line-status decision $z_\ell$ is realized by device commands $\Phi_\ell$:

$$z_\ell=u_{sw}\ \text{(switch/tie)},\quad z_\ell=u_{cb}\ \text{(single-side)},\quad z_\ell=u_{\text{from}}\wedge u_{\text{to}}\ \text{(double-side)}.$$

## 2. Variables

Per edge $e$ and bus $i$: $\beta_e\in\{0,1\}$ status; $P_e,Q_e$ flows; $v_i=|V_i|^2$; $f_e$ commodity flow; $\gamma_g\in\{0,1\}$ root; $s_i^P,s_i^Q\ge0$ shed; $t_e\ge0$ for $|P_e|$.

## 3. Constraint groups (GUI-toggleable)

- **G1 Tree cardinality:** $\sum\beta_e+\sum\gamma_g=n_b$.
- **G2 Connectivity (commodity flow):** root $\sum f-\sum f=-(n{-}1)$, others $=1$, $|f_e|\le n_b\,\beta_e$.
- **G3 Power balance:** $\sum_{\text{in}}P-\sum_{\text{out}}P-P_g=-P_i^{net}-s_i^P$ (Q similar, AC).
- **G4 Voltage drop (LinDistFlow, big-M):** $|v_j-v_i+2r_eP_e+2x_eQ_e|\le M(1-\beta_e)$.
- **G5 Thermal:** $|P_e|\le P_e^{\max}\beta_e$, $|Q_e|\le Q_e^{\max}\beta_e$.
- **G6 VSC transfer:** $|P_e^{vsc}|\le S^{\max}$, reactive approximated.
- **G7 Switch budget:** $\sum|\beta_e-\beta_e^0|\le N_{sw}$.

G3–G6 only when **PF optional** is on; G1–G2 always (pure connectivity). G4/G5 are independently toggleable (`enable_voltage`, `enable_thermal`). G7 active when `max_switch_ops>0`.

## 4. Objective (GUI-configurable weights)

$$\min\ \lambda_{sw}\!\sum_{\text{ties}}\!\beta_e-\lambda_{sw}\!\sum_{\text{in-svc}}\!\beta_e+\lambda_{loss}\!\sum r_e\beta_e+\lambda_{shed}\!\sum(s^P+s^Q)+\lambda_{isl}\!\sum_{g>0}\gamma_g.$$

Toggle: min-loss ($\lambda_{loss}$), min-switching ($\lambda_{sw}$), max-restored ($\lambda_{shed}$), min-islands ($\lambda_{isl}$).

## 5. Optional power flow

PF off → G1/G2 only (connectivity, fast). PF on → LinDistFlow G3–G6 (voltage/thermal aware).

## 6. Post-optimal PF cross-check

Apply $\beta^\*$, run full Newton PF; report radial/connected/islands, true loss, CB flows via `compute_device_terminal_flows`. Flags `post_power_flow_validated`/`full_hybrid_opf_validated`.

## 7. Theory: do AC/DC converters need tree constraints?

**No — VSC/DC-DC must be excluded from radiality.** Radiality is per electrical domain. A VSC bridges an AC and a DC node; closing it never forms an AC loop, so counting it in $\sum\beta=n_b{-}1$ over-constrains. Correct: per-domain trees + converters as controllable power transfers, $|P^{vsc}|\le S^{\max}$, $z=1$ for healthy converters. Use $\sum_{AC}\beta=n_{ac}{-}1$, $\sum_{DC}\beta=n_{dc}{-}1$, converters free. Enabled via `split_domain_trees`: VSC β dropped from cardinality so meshed MTDC links stay closed.

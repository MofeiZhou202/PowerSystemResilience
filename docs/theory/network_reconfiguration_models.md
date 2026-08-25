> Documentation sync: 2026-08-21
> Status: implementation-backed quick reference.
> Complete contract: [`docs/modules/network_reconfiguration/network_reconfiguration_manual.tex`](../modules/network_reconfiguration/network_reconfiguration_manual.tex).

# Network Reconfiguration Models in Canonical Space

The maintained entry is `analysis::run_topology_reconfiguration(const
HybridPowerSystem&, const TopoReconfOptions&)`. It solves one snapshot in
canonical AC/DC/VSC edge space. The historical
`solve_optimal_reconfiguration` API is now an AC compatibility wrapper around
that entry; the older inline AC B&C body below its unconditional return is not
reachable.

## 1. Projection and identifiers

The core projects with `strip_dead_islands=false` and
`preserve_switch_branches=true`. AC and DC buses use separate ID maps and are
combined only as local positions. Hybrid edge inputs and outputs use
`BranchRef { EdgeCategory, component.index }`; legacy bare-integer vectors are
ambiguous when AC, DC, and VSC components share an index.

Canonical edges are

$$E=E_{ac}\cup E_{dc}\cup E_{vsc}.$$

`BranchExpandMap` attributes changed canonical AC edges back to rich switches
or circuit breakers. Device capabilities, locks, fuse restrictions, and
upstream-protection bindings can remove a requested edge from the candidate
set. DC and VSC candidates are branch-level decisions.

## 2. Variables

The always-present topology block contains signed fictitious flows $F_e$,
root injections $F_g$, edge states $\beta_e$, and root indicators $\gamma_g$.
With `enable_pf=true`, the model adds line/VSC $P,Q$, source $P_g,Q_g$,
squared voltages $v_i$, and nonnegative shedding $s_i^P,s_i^Q$. With
`loss_aware && enable_pf`, it also adds $t_e\ge |P_e|$ for AC/DC lines.

## 3. Topology constraints

Unified mode uses source-rooted fictitious-flow balance and

$$\sum_{e\in E_{participating}}\beta_e+\sum_g\gamma_g=n_b.$$

Split-domain mode fixes VSC fictitious flow to zero and imposes a separate
tree-edge count for each precomputed AC component and, unless
`allow_dc_mesh=true`, each DC component. VSCs do not count as AC or DC tree
edges. `allow_dc_mesh` therefore does not certify DC radiality.

Faulted edges are fixed open. Non-candidates keep their authored state. A
switch-operation budget expands $|\beta-\beta^0|$ directly and uses device
operation costs; a de-energized isolation sequence conservatively costs three
actions.

## 4. Optional electrical layer

With PF enabled, active balance covers AC and DC buses; reactive balance covers
AC buses. The line voltage approximation is

$$|v_j-v_i+2r_eP_e+2x_eQ_e|\le M_e(1-\beta_e),$$

with no $Q$ term on DC lines. Thermal limits are independent boxes on $P$ and
$Q$, not circular MVA limits. VSC active flow enters both terminals with no
efficiency loss; VSC reactive flow is an AC-terminal approximation. The model
has no current-squared variable or branch-loss term in balance.

AC bus-level demand and `Load` rows are additive. DC bus-level demand and
`DCLoad` rows are additive. Source limits aggregate generators, DER, storage,
external grids, DC sources, and eligible DC-voltage buses. Zero-capacity
pseudo-roots allow an isolated section to remain topologically represented
while PF balance charges its load as shedding.

## 5. Objective and loss meanings

The objective is a weighted sum of switching, a resistance/flow proxy,
shedding, and extra roots. Initially closed switching terms omit a constant,
so `milp_objective` is not the sum of the positive `obj_terms` report and has
no MW unit.

- `obj_terms.loss`: weighted objective contribution.
- core `reconf_loss_mw`: $base\_mva\sum_{closed\ AC/DC}r_e$, a nominal 1 pu
  current topology proxy, not solved loss.
- HTTP `reconfig_loss_mw`: post-power-flow branch loss, meaningful only when
  `reconfig_pf_converged=true`.

The HTTP field `estimated_loss_mw` now returns the core proxy `reconf_loss_mw`
(a defined nominal-current MW proxy), not the old dimensionally invalid
`milp_objective * base_mva` (AUD-018, fixed). For physical loss use
`reconfig_loss_mw` when `reconfig_pf_converged=true`.

## 6. Solver and certificate boundary

The graph heuristic can return a feasible incumbent but never an optimality
certificate. Reachable backend dispatch is:

- `native`: Native B&C only;
- `scip`: SCIP only;
- `highs` or `auto`: HiGHS, then SCIP on failure;
- unknown strings: same as `auto`.

Every returned vector is checked against linear equalities, inequalities,
integrality, and bounds. `feasible` means only that this linear model passed.
The core does not run full hybrid PF/OPF. The production HTTP route applies
actions to the rich model, reprojects, runs PF and AC OPF, and only then builds
its `executable` flag.

The compatibility `ONRResult` runs a verification AC PF after a core incumbent,
but PF non-convergence does not clear its optimization `feasible` flag. It also
contains a fallback that can report `feasible=true` for the unchanged original
topology when the core solve failed and the base PF converged; no result field
currently identifies that fallback.

## 7. Explicit limitations

This is a single-snapshot, balanced steady-state approximation. It does not
implement exact DistFlow/SOCP/AC reconfiguration, three-phase switching,
multi-period storage and switching schedules, N-1/stochastic ONR, dynamic
protection, communication failures, or a complete hybrid nonlinear PF/OPF
certificate inside the core entry.

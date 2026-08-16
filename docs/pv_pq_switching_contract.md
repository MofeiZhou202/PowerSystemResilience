# PV/PQ Reactive-Limit Switching Contract

This document defines the current balanced Newton power-flow behavior for
generator reactive limits. The implementation is in
`src/power_flow/newton_solver.cpp`; the public options and result diagnostics
are in `include/hacdcpf/power_flow/`.

## Why unrestricted switching is expensive

For a fixed PV/PQ active set `A`, Newton solves a smooth system

```text
F_A(theta, V) = 0.
```

In the legacy reduced layout, changing a PV bus to PQ changes the voltage
variables, Q equations, sparse Jacobian pattern, symbolic analysis, and numeric
factorization. If switching
is performed from unconverged intermediate iterates, the implied generator Q
is not yet an accurate limit test. Bilateral PV-to-PQ and PQ-to-PV decisions
can then revisit the same active set. The cost is dominated by repeated sparse
factorizations rather than by the Q comparisons themselves.

The implementation therefore treats Q-limit handling as a bounded
primal-dual active-set process around fixed-layout Newton solves:

1. Solve the current fixed active set to the requested electrical residual.
2. Evaluate generator Q at that converged point.
3. Move every violated PV bus to its exact Q bound in one batch.
4. Repeat until no PV bus violates a bound.
5. Perform at most one PQ-to-PV restoration audit.
6. Re-clamp any restored bus that violates Q, without a second restoration.

Batch entry is important. An entry dead band can hide a physical Q violation
and can split one active-set update into several costly Newton solves. The
hysteresis is applied only to release: a limited bus must satisfy the minimum
hold count and the voltage-error direction test before restoration. This is a
Schmitt-trigger policy on release, while entry remains an exact feasibility
test.

The monotone entry phase can add each originally controlled PV bus at most
once. The single restoration audit prevents an unbounded two-cycle. The outer
loop is additionally limited by `pv_pq_max_outer_iterations`; exhaustion is an
explicit uncertified result, not a silent success.

This policy follows the finite-identification rationale of primal-dual active
sets; see Hintermueller, Ito, and Kunisch, *SIAM Journal on Optimization*,
13(3), 2002. It is adapted here to the expensive sparse-pattern changes of AC
power flow.

## Fixed superset Jacobian layout

`RobustNonlinearOptions::enable_fixed_pv_pq_layout` is the default structural
layout for the same outer active-set policy. It allocates one voltage
magnitude column and one reactive-balance row for every non-slack, non-reference
AC bus, in stable canonical bus-position order. At each residual/Jacobian
evaluation:

- a currently active PV bus replaces its Q-balance row with
  `Vset - Vm = 0` and an identity `Vm` column;
- a Q-limited PQ bus retains the physical Q-balance row;
- PV-to-PQ and the bounded restoration audit still happen only at a converged
  outer point, so the certificate and switch semantics are unchanged.

The stable bus-position order is part of the layout contract. Grouping current
PV buses before current PQ buses would permute coordinates after a partial
switch and would still invalidate symbolic analysis. The implementation
precomputes compressed nonzero indices by row, so row replacement is
`O(nnz(row))`; the symbolic pattern is built and analyzed once per solve. The
option is mutually exclusive with semi-smooth NCP: when both are requested,
NCP owns the Q-row equations and the fixed-layout request is ignored with an
explicit diagnostic warning.

The fixed layout is algebraically an embedding of each existing fixed-active-set
Newton system, not a new Q-limit model. PV identity targets follow the current
Newton voltage state: either the authored/caller initial value or the `Vg`
installed by a PQ-to-PV restoration. This matches the value held outside the
reduced system rather than silently changing the fixed-active-set equation.

## Ideal connectivity must be removed before switching

Active-set safeguards cannot repair an inconsistent network representation.
If an artificial tie with reactance `x` connects two PV controllers whose
voltage setpoints differ by `Delta V`, its reactive circulation scales as
`O(Delta V / x)`. Treating a BPA ideal connection as a physical `x=1e-4 pu`
line can therefore create thousands of Mvar of artificial Q demand. PV-to-PQ
conversion then correctly reports a large violation, but no choice of
hysteresis or outer-iteration limit can make the inconsistent voltage controls
feasible.

Canonical projection contracts `ACBranch::ideal_connectivity` rows before
Ybus construction and before the PV/PQ active set is formed. This is the
quotient graph `V/~` of electrically identical nodes: extensive injections and
controller limits are aggregated on the canonical bus, and solved voltages are
broadcast back through `BusMergeMap`. The contraction is provenance based.
BPA exact-zero L cards carry explicit provenance, while a narrowly scoped
legacy-JSON migration recognizes their old `1e-4 pu` numerical encoding.
An unmarked short physical line is never contracted merely because its
impedance is small.

## Complementarity alternative

Semi-smooth Newton retains one equation layout and represents the PV/PQ/Q-limit
branches with the median equation

```text
max(min(V - Vset, Qg - Qmin), Qg - Qmax) = 0.
```

Its zero set contains the three physical cases: lower Q bound, voltage control
inside the Q interval, and upper Q bound. When smooth continuation is enabled,
CHKS-smoothed min/max operators replace the nonsmooth operators. For example,

```text
max_mu(a,b) = 0.5 * (a + b + sqrt((a-b)^2 + 4*mu^2)).
```

`mu` is initialized before the first residual/Jacobian evaluation. A reduction
of `mu` always forces a fresh residual and Jacobian assembly; the solver never
uses a Jacobian from the previous continuation level. Convergence is accepted
only after `mu` reaches `ncp_mu_min`. The result reports the number of
continuation updates and final `mu` for audit. This is the Chen-Harker-Kanzow-
Smale smoothing construction applied to the median NCP.

## Default numerical escalation

The public balanced PF facade escalates only a numerically failed,
structurally closed solve. The order is fixed:

1. direct fixed-layout active-set Newton;
2. semi-smooth NCP from the caller/authored state;
3. linearized-DC AC-angle seed plus fixed-layout active-set Newton;
4. the same DC-angle seed plus semi-smooth NCP;
5. homotopy continuation as the final fallback.

The linearized-DC stage supplies only AC angles. AC and DC voltage magnitudes
come from the canonical authored profile. Each direct retry uses private solver
data and a private Newton solver, so NCP/active-set state cannot alter the
facade's cached pattern. If the caller disabled generator-Q enforcement, NCP
stages are skipped rather than silently changing the requested model. LCC tap
control retains its dedicated outer loop. Successful results report
`successful_fallback_stage`; attempted NCP, DC seed, and homotopy stages and
their aggregate work remain visible in `SolverProfiling` and runtime JSON.

The five-case flat-start acceptance suite is `case1888rte`, `case3375wp`,
`case6468rte`, `case6515rte`, and `case_ACTIVSg10k`. Release/KLU verification
converged and Q-certified all five without reaching homotopy: `case3375wp`
used NCP, and the other four used the DC-angle/fixed-active-set stage.

## Result semantics

`PowerFlowResult::reactive_limits` is the authoritative certificate:

| Field | Meaning |
|---|---|
| `enforcement_requested` | Active-set conversion or semi-smooth NCP was requested. |
| `certified` | The electrical solve converged, the outer budget was not exhausted, and the final generator Q violation is within the certificate tolerance. |
| `active_set_cycle_detected` | A converged active-set signature was revisited. |
| `outer_iteration_limit_reached` | The bounded outer work budget was exhausted. |
| `active_limited_buses` | Number of buses held at a Q limit by active-set conversion. |
| `max_violation_pu` | Maximum final generator Q-bound violation on system base. |

The certificate tolerance is `max(1e-10, PowerFlowOptions::tol)`. At a bus
held on a reactive limit, its apparent remaining Q violation is the final Q-row
mismatch, so a certificate cannot honestly claim accuracy tighter than the
root used to compute it.

When enforcement is disabled, convergence is an electrical power-flow result,
not a generator-Q-limit certificate. The GUI therefore defaults to the faster
screening mode with `enable_pv_pq_conversion=false`; engineering checks that
require Q feasibility must enable `Q限值校核（PV→PQ）` and inspect
`reactive_limits.certified`.

## Cost model and measured boundary

For a large sparse case, the relevant wall-time model is

```text
T = Nfactor * Cfactor(pattern) + Nsolve * Ctriangular + Neval * Ceval.
```

Preventing extra active-set batches matters more than making the scalar Q-limit
test cheaper. With ordinary reduced layouts, the Release/KLU `case6515rte`
regression used 3 pattern rebuilds/analyses, 9 Newton iterations, 8
factorizations, one 74-bus PV-to-PQ batch, no restoration, and a five-repeat
median of `58.7 ms`. Enabling the fixed layout kept the same switch trajectory,
iterations, factorizations, certificate, and final voltages while reducing the
pattern rebuilds/analyses to `1/1` and the median to `40.9 ms` (`30%` lower).
On generator-rich `case_ACTIVSg10k`, rebuilds/analyses fell `10/10 -> 1/1`
with the same 26 iterations, 25 factorizations, 1151 entries, 40 restorations,
and certified Q limit; the five-repeat median fell `261.1 -> 152.0 ms`
(`42%`). Measurements used AppleClang 21 arm64 macOS, Release/KLU, at working
tree `3cc92658373a`; they are a local regression boundary, not a portable
hardware guarantee.

With the fixed pattern in place, KLU numeric refactorization reuses the prior
pivot order between Newton iterations. Each candidate is accepted only when
its normwise backward error is within the configured tolerance; otherwise the
solver performs a fresh numeric factorization with pivoting. A five-repeat
Release protocol on warm `case_ACTIVSg10k` reduced median total solve time from
`154.032` to `104.024 ms` and final-solve linear time from `98.549` to
`54.699 ms`; all 24 candidates were accepted with maximum backward error
`2.29e-17`. These measurements use the pinned MIPSolvers `3bf1e66` KLU adapter.

Repeated compatible solves can use `PreparedPowerFlowSession` to retain the
canonical projection, assembled matrices, fixed pattern, and symbolic analysis.
Load/setpoint-only changes refresh numeric data; topology or network-parameter
changes trigger a conservative rebuild. This is an explicit lifecycle API, not
an unsafe implicit cache behind `solve_power_flow`.

The original `data/云南案例.json` has 4512 authored AC buses. Its legacy BPA
encoding contains 587 provenance-recognized numerical ties, producing 585
effective bus merges and a 3927-bus canonical network. Before contraction,
Q-limit enforcement stopped uncertified at about `410 pu` electrical residual
and `8736 pu` maximum Q violation; there was no repeated active set. After
contraction, the exact GUI request converged in 19 total Newton iterations and
5 active-set solves, with 218 PV-to-PQ entries, 3 restorations, no cycle, and a
certified `2.27e-13 pu` maximum Q violation. The cold solver and complete
pre-serialization response measured 71.9 ms and 296.8 ms respectively; five
warm compact solves had a 47.3 ms median. This is evidence for model reduction
before active-set tuning, not a portable timing guarantee.

The GUI compact response avoids large authored-space attribution payloads, but
does not change the numerical certificate. Adaptive and hybrid-linearized
methods remain screening algorithms unless their returned validity fields
explicitly certify the same nonlinear Q-limit model.

For disconnected pure-AC systems, the GUI `ac_newton` route uses the adaptive
island solver once and aggregates each island's Q-limit certificate, switch
counts, repeated-set flags, and work counters. It must not run a second
monolithic Newton solve merely to construct presentation data. Branch flows are
reconstructed directly from the merged island voltage result.

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

Changing a PV bus to PQ changes the voltage variables, Q equations, sparse
Jacobian pattern, symbolic analysis, and numeric factorization. If switching
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
test cheaper. On the Release `case6515rte` regression, fast screening used 4
Newton iterations and 3 factorizations with a repeated-solve median of about
21.2 ms. Certified Q-limit handling used 9 iterations, 8 factorizations, one
74-bus PV-to-PQ batch, no restoration, no repeated active set, and a repeated-
solve median of 50.6 ms in the latest five-repeat run. The certificate
violation was `4.22e-11 pu`. These values are a local regression boundary, not
a portable hardware guarantee.

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

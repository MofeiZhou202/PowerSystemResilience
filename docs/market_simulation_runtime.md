# Native Day-Ahead Market Runtime

The first native electricity-market vertical slice is implemented by
`hacdcpf::market::run_day_ahead_market`.  It operates on an immutable
`HybridPowerSystem` and a `TimeSeriesData` horizon:

```text
participant behaviour and submitted offers
  -> network-constrained SCUC
  -> fixed-commitment multi-period SCED
  -> DC-storage intertemporal co-optimization
  -> preventive AC-branch cuts + corrective full-component N-1 checks
  -> nodal LMP and upward-reserve price
  -> base-case nonlinear hybrid AC/DC power-flow certification
  -> day-ahead settlement and make-whole uplift
  -> fixed-commitment real-time SCED on realized profiles
  -> day-ahead schedule + real-time deviation two-settlement
  -> reserve activation, performance, and imbalance settlement
```

## Pricing contract

- Energy offers are convex piecewise-linear secants of each generator's
  `cost_c2 * p^2 + cost_c1 * p + cost_c0` physical cost curve.
- Market SCUC co-optimizes commitment, dispatch, submitted energy blocks and
  upward reserve.  Its objective contains accepted energy blocks, minimum-
  output/no-load prices, startup and shutdown prices, reserve offers, lost-load
  cost, and exogenous-injection curtailment cost.  It enforces state
  transitions, minimum up/down times, daily startup/shutdown limits, initial
  and inter-period ramps, reserve deliverability, DC branch flows and thermal
  limits.  In a hybrid case it also models DC-bus voltage, lossless resistive
  DC-branch transfer, and bidirectional VSC/DC-DC transfer with constant
  efficiency. SCED fixes commitment and converter directions and retains the
  same physical dispatch envelope. Eligible DC storage is co-optimized with
  charge/discharge direction, efficiency, self-discharge, SOC bounds, optional
  terminal-SOC recovery, and per-day throughput limits; SCED fixes its SCUC
  direction while retaining continuous dispatch and SOC.
- AC and DC LMPs are duals of their domain-qualified nodal active-power
  balances.  The upward-reserve price is the dual of the system
  reserve-balance row.
- All dispatch quantities use MW, interval energy uses MWh, and prices use
  currency/MWh.  `TimeSeriesData::step_duration_hr` scales objectives,
  revenues, and costs.

## Settlement contract

`GeneratorSettlement` keeps submitted offers separate from physical true cost.
Energy and reserve clear pay-as-clear.  A committed generator receives
make-whole uplift when its energy and reserve revenue do not recover quadratic
production, no-load, startup, and shutdown cost.  The ledger reports the
identity

```text
customer total payment
  = resource total revenue + congestion rent + cashflow residual
```

and `cashflow_residual` must be numerically zero.

`as_bid_cost` is reconstructed from accepted offer blocks, submitted no-load /
startup/shutdown prices, and reserve offers.  `true_cost` is evaluated from the
immutable physical generator cost curve.  Make-whole uplift recovers
`as_bid_cost`; participant profit is revenue plus uplift minus `true_cost`.

Customer energy charges use served demand (`gross demand - load shedding`), so
unserved MWh are not charged at the energy LMP.  Fixed renewable/static/storage
injections are paid only for delivered injection after nodal curtailment.  This
keeps load shedding, renewable curtailment, congestion rent, and the cash-flow
identity separate and auditable.

Eligible optimized DC storage is settled separately at its authored DC-bus LMP
on net injection (discharge minus charge). `DCStorageSettlement` reports stable
component identity, charge/discharge energy, initial/terminal SOC, energy
revenue, as-bid cost, and profit. It receives no uplift or reserve award.

## Participant behaviour

`MarketParticipant` owns one or more generator positions and submits offers
through one of four deterministic benchmark policies:

| Policy | Energy markup | Capacity withholding |
|---|---:|---:|
| `cost_based` | no | no |
| `fixed_markup` | yes | no |
| `capacity_withholding` | no | yes |
| `markup_and_withholding` | yes | yes |

Withholding applies only to flexible capacity above minimum stable output.
Submitted capacity, explicit marked-up energy blocks, commitment prices, and
reserve prices enter both SCUC and fixed-commitment SCED.  The physical system
is never mutated.  Unowned active generators receive explicit cost-based
default participants, while duplicate ownership and invalid positions are
rejected.

The result includes a per-generator `BehaviorAction`, participant-level
settlement, output/revenue HHI, Top-3 output share, maximum participant profit,
total withheld capacity, and average submitted markup.

## Nonlinear hybrid certification

Each commercial SCED interval is replayed through the native hybrid AC/DC
power-flow solver after applying AC/DC nodal load shedding, AC/DC exogenous
curtailment, and converter schedules to the physical snapshot.
`ACValidationPeriod` reports convergence, residual, physical branch loss,
AC/DC voltage violations, AC apparent-power overload, physical DC-branch
overload, converter schedule deviation, and the slack adjustment required to
supply physical losses. One online slack generator is selected per connected
AC component. A converged power flow is only `secure` when all configured
voltage, thermal, and generator tolerances pass; otherwise the market returns
`ac_validation_failed`. The commercial schedule is preserved; certification
adjustments are not silently written back into settlement.

## Full-component N-1 security

When `enable_n1_security` is true, the runner first preserves an unconstrained
SCED benchmark and constructs branch outage distribution factors (LODF) for
the connected AC network.  Outages that would island the network are skipped
and reported explicitly.  Every screened violation has the form

```text
abs(base monitored flow + LODF * outage-branch flow) <= emergency rating
```

The violated direction is added to SCED as a linear cut, and screening repeats
until no violation remains or an iteration/cut limit is reached.  Since these
cuts are rows in the final pricing LP, their congestion effects are reflected
in the final LMPs.  `SecurityResult` records the initial and final violation
counts, cut trajectory, remaining violations, skipped islanding outages, and
the preventive redispatch cost relative to the unconstrained benchmark.

With `run_ac_contingency_validation`, selected outages are also replayed using
the nonlinear AC solver at the final preventive dispatch.  Each result reports
convergence, voltage-limit violation, and emergency thermal overload.  A
failed AC contingency remains a failed certification and makes the requested
market run infeasible; the validator does not silently redispatch it.

Commercial AC-branch screening treats the MVA rating as an active-power MW
limit. The subsequent nonlinear certification uses apparent power and is
therefore the physical AC thermal check.

The same N-1 request also adds a preventive generator-capability constraint to
SCUC, then enumerates every in-service AC generator, AC branch, DC branch, VSC,
DC/DC converter, legacy DC storage, and rich DC storage. For each outage it
holds the accepted commitment and converter/storage directions fixed and solves
one corrective multi-period SCED with the component unavailable for the entire
market horizon. Any SCED failure or incremental load shedding makes that
contingency insecure. `n1_max_contingencies=0` means full coverage; a positive
value truncates the candidate sets.

This is deliberately a mixed recourse policy: only preventive AC-branch LODF
cuts are rows of the pricing LP. Generator capability is procured in SCUC, and
the corrective component checks are certifications rather than price-forming
constraints. Bus, load, shunt, switchgear, and protection failures remain
outside this market N-1 element set. Optional nonlinear outage replay remains
AC-branch-only.

## Current scope boundary

This slice supports AC buses/branches/generators, DC buses/branches/loads and
fixed resources, plus VSC and DC/DC transfer. Every energized metallic DC
island must have a `DC_V` bus, a DC-voltage-forming VSC, or an output-side
Voltage/Droop DC/DC control. In-service external grids and energy-router ports
remain explicitly rejected.

The commercial DC-branch model is voltage-linearized and lossless. VSC and
DC/DC constant directional efficiency is modeled, but VSC fixed/quadratic
losses, current/capability/modulation limits, converter bids, and price-forming
DC/converter contingency cuts are not. AC storage remains exogenous. Optimized
DC storage does not provide reserve or receive uplift. `MarketModelScope`
exposes these boundaries and the N-1 recourse policy in every result.

## Scalability and performance contract

Every day-ahead result includes `MarketPerformanceProfile`: active component
counts, exact SCUC/SCED variable-count estimates, binary-variable count, a
legacy dense-LODF baseline, sparse working-memory estimate, actual computed
LODF column count, selected solver names, pricing fallback status, contingency
LP count/worker count, and wall-clock timings by stage.
Pricing LPs with at most `pricing_native_max_variables` variables first use the
native simplex and automatically retry with embedded HiGHS when the native
Phase I or numerical path fails. Larger LPs route directly to HiGHS and skip
the known-expensive native attempt; the default threshold is 5000 and zero
means direct HiGHS for every pricing LP. HiGHS row duals remain the source of
reported LMPs. The profile distinguishes direct routing from fallback.

Large-system use remains bounded by three implementation facts. Generator N-1
capability uses one total-reserve auxiliary per period, reducing its matrix
nonzeros from `O(T * G^2)` to `O(T * G)`. The LODF builder now
factorizes the reduced bus matrix once with sparse LU and stores only requested
outage columns; full coverage still stores `L x L` values. Full-component
corrective validation rebuilds and solves one multi-period SCED per candidate,
using bounded task parallelism with deterministic authored-order reporting.
Each worker owns a model and solver instance, so worker count trades elapsed
time for peak memory. Consequently, an unrestricted 24-hour, full-component
N-1 provincial run is still not a production latency contract.

Only generator commitment states remain binary. Startup and shutdown
auxiliaries are continuous in `[0,1]`: binary adjacent commitment states and
the transition equality force every real transition, while an extra
simultaneous startup/shutdown can only add non-negative cost and tighten the UC
constraints. This reduces generator binaries from `3 * T * G` to `T * G`
without changing the projected optimal commitment/dispatch contract.

The market builder now supplies SCUC structure rather than submitting an
anonymous MILP. `UCGenHint` describes commitment/transition/dispatch columns,
unit limits, time coupling, reserve and offer segments. A continuous relaxation
is rounded with load, reserve, contingency-capacity and merit-order knowledge;
the integer choices are then fixed and the complete network LP is solved. Only
a successful fixed-integer LP is passed as `initial_solution`. This warm start
is skipped for one-period cases, where there is no temporal chain to exploit.
Commitment
variables receive higher branching priority than converter/storage directions;
earlier periods and larger units rank first.

For `highs`/`auto`, models at or above
`structured_scuc_min_binary_variables=1000` use StrictHiGHS with the supplied
priorities, root separation, reliable pseudocosts, symmetry detection and the
verified incumbent. Smaller models retain lightweight HiGHS. The default
`scuc_mip_relative_gap` is 0.01 and `scuc_time_limit_sec` is 120. Results expose
the actual gap, solver termination, whether the target gap was met, and whether
near-zero-gap optimality was proven; these states are not interchangeable.

Large base-network SCUCs use exact AC thermal-limit constraint generation when
the finite branch-period candidate count reaches
`scuc_network_constraint_generation_min_candidates` (default 10000). Every
nodal balance and branch flow equation remains in the master MILP; only finite
AC flow bounds are initially relaxed. After each MILP solve, every omitted
bound is scanned, violations are sorted by MW magnitude, and the
violated bounds are restored. A fixed-integer LP repairs the incumbent before
the next iteration.
The run is accepted only when no omitted bound exceeds the configured tolerance.
An iteration/time-limit exit with remaining violations returns infeasible and
does not proceed to pricing. DC branch limits remain present in every master.
The last restricted-master lower bound is also a valid lower bound for the full
model, so its reported gap remains conservative once the incumbent passes the
complete thermal scan.

Before the first MILP, the feasible MIP-start candidate receives the same full
thermal scan. Violated physical bounds are activated immediately and the fixed
integer assignment is repaired once against that tighter master. This reuses
work already paid for by warm-start construction to avoid a known-loose first
MILP without weakening the final certificate.

For structured StrictHiGHS, omitted thermal bounds are first submitted inside
the active branch-and-bound tree as equivalent angle-difference global cuts.
The callback runs after a node LP is solved and before incumbent acceptance,
pruning or branching. Each branch-period limit is submitted once per solve;
the reported submission count is not presented as an acceptance count. The
returned incumbent still receives the complete external scan. If the row
cannot be projected exactly through presolve or is not enforced, the violated
physical bound is restored and the exact outer loop remains the fallback.
Other MILP backends use the outer loop directly.

Structured StrictHiGHS rounds reuse original-space root cuts, the paired root
simplex basis, and original-column pseudocost statistics. Tightening omitted
flow bounds only shrinks the feasible set, so globally valid cuts from the
previous master remain valid. Basis and pseudocost payloads are accepted only
after exact dimension checks; missing or incompatible artifacts are ignored.
The open search tree is rebuilt because tightened bounds invalidate stored node
relaxations and bounds, and the current HiGHS API has no node-by-node certified
tree checkpoint. Performance output distinguishes artifact reuse from tree
rebuilding.

For controlled large cases, run base SCUC/SCED first, apply a nonzero
`n1_max_contingencies` budget, and perform the complete corrective sweep as an
offline batch. The default corrective-SCED worker cap is four;
`component_n1_parallel_workers=0` explicitly selects hardware concurrency and
should only be used after measuring memory. HiGHS or SCIP should be preferred
over native B&C. Job cancellation/progress, rolling-horizon decomposition,
cross-run artifact caches, persistent search trees, and registered scaling budgets remain required
before claiming province-scale online readiness.

Current release-build reference runs show the intended direction: a one-period
ACTIVSg2000 base market (432 active generators, 3206 branches, one offer block)
dropped from about 28.52 s to 4.05 s after direct large-LP routing; a ten-candidate
N-1 run dropped from about 26.29 s to 7.97 s after the linear-size generator
capability reformulation. These are local development measurements, not an SLA.
On the same build, a 56-generator ACTIVSg500 24-period run that exceeded 120 s
at a `1e-3` target completed in about 2.13 s at an explicit `1e-2` target; its
reported final gap was about 0.760%, so it met the configured target but did
not constitute a zero-gap proof.
The pre-reuse baseline with AC thermal limits enabled on the 24-period
ACTIVSg2000 case generated only 34 active bounds from 76,944 candidates, but a
second master restart exhausted the shared 120 s budget (120.92 s measured).
The current path separates the feasible MIP start before the first MILP: the
same 34 bounds were activated, the final incumbent passed all 76,944 thermal
checks, and no outer tree rebuild was needed. Even so, the 284,520-variable,
10,560-binary StrictHiGHS solve consumed 120.83 s without reporting a gap that
met the 1% target. It therefore still refused pricing. Cross-round root cuts,
basis and pseudocosts remain available when another exact outer round is
needed, but this benchmark now identifies the single large SCUC solve, rather
than network-constraint restart, as the limiting stage.

## Real-time market and deviation settlement

`run_real_time_market` accepts the immutable physical system, the day-ahead
time series/result, and a realized time series with the same horizon.  The
accepted day-ahead unit commitment is pinned through the existing fixed-
commitment SCED contract; real-time clearing can still use network constraints,
N-1 cuts, and AC certification.

The commercial settlement is the standard two-settlement identity:

```text
generator revenue
  = day-ahead scheduled MWh × day-ahead LMP
  + real-time deviation MWh × real-time LMP
  + accepted day-ahead reserve revenue and uplift

customer payment
  = day-ahead payment
  + realized demand deviation × real-time LMP
```

Actual generator profit uses the realized dispatch true cost, not the
day-ahead scheduled cost.  Exogenous renewable/static-injection deviations are
included in resource deviation revenue. Optimized DC storage contributes its
real-time-minus-day-ahead net-injection deviation at the real-time DC LMP; the
day-ahead and real-time storage optimization policies must match. These terms
enter the deviation congestion-rent identity. `DeviationSettlementLedger::cashflow_residual`
audits the combined day-ahead and real-time cash flow.

### Real-time ancillary services and imbalance charges

When `RealTimeMarketOptions::ancillary_services.enabled` is true, upward net-
demand deviation activates the accepted day-ahead upward-reserve awards.  The
activation instruction is allocated pro rata and capped by each award.  A first
fixed-commitment SCED produces the ISO dispatch instruction while enforcing
that activation.  Performance factors then convert the instructed response to
actual output.  A second fixed-commitment balancing SCED pins non-performing
units at actual output and redispatches the remaining fleet to replace the
shortfall.  Real-time clearing does not procure a second unpaid reserve
product.

Delivered reserve earns the configured performance payment; shortfall pays the
non-performance charge.  Generator imbalance is measured as actual output
minus the ISO real-time instruction, so ordinary ISO redispatch relative to the
day-ahead schedule is not misclassified as non-performance.  Nodal load
deviations outside their day-ahead tolerance bands pay separate imbalance
charges.  The combined cash identity is:

```text
customer two-settlement payment
  = resource two-settlement revenue
  + total congestion rent
  + system-operator ancillary balance
  + cashflow residual

system-operator ancillary balance
  = load imbalance penalties
  + generator imbalance charges
  + reserve non-performance charges
  - reserve performance payments
```

The resource revenue and participant profit used by the repeated game include
the net ancillary adjustment automatically.

## Repeated participant game

`run_repeated_market_game` runs repeated day-ahead and optional real-time
markets.  Learning participants use synchronous bounded local best responses:

1. clear the current strategy profile and calculate two-settlement profit;
2. for each participant, hold opponents fixed and evaluate one step up/down in
   energy markup and capacity withholding;
3. accept only a candidate that improves that participant's profit by more
   than the configured tolerance;
4. apply all accepted responses simultaneously and repeat.

This is a deterministic benchmark game rather than an opaque learned policy.
Each round records current/next behaviour, current profit, best-response
profit, LMPs, generator deviation, and HHI.  The game reports convergence when
no configured participant has a profitable local deviation.

## HTTP endpoint

`POST /api/session/run_market_clearing` accepts:

```json
{
  "num_steps": 24,
  "offer_segments": 8,
  "uc_solver": "highs",
  "structured_scuc_branching": true,
  "structured_scuc_min_binary_variables": 1000,
  "scuc_mip_relative_gap": 0.01,
  "scuc_time_limit_sec": 120,
  "scuc_max_nodes": 50000,
  "enable_scuc_cross_round_solver_state_reuse": true,
  "enable_scuc_in_solve_network_constraint_generation": true,
  "enable_scuc_network_constraint_generation": true,
  "scuc_network_constraint_generation_min_candidates": 10000,
  "scuc_network_constraint_generation_max_iterations": 8,
  "scuc_network_constraint_generation_max_new_per_iteration": 0,
  "scuc_network_constraint_generation_tolerance_mw": 1e-5,
  "pricing_native_max_variables": 5000,
  "reserve_fraction": 0.05,
  "voll_per_mwh": 10000.0,
  "exogenous_curtailment_penalty_per_mwh": 5.0,
  "optimize_dc_storage": true,
  "enforce_terminal_dc_storage_soc": true,
  "network_constraints": true,
  "run_ac_validation": true,
  "enable_n1_security": true,
  "n1_max_iterations": 8,
  "n1_max_cuts_per_iteration": 200,
  "n1_max_contingencies": 0,
  "parallel_component_n1": true,
  "component_n1_parallel_workers": 4,
  "n1_emergency_rating_multiplier": 1.0,
  "run_ac_contingency_validation": true,
  "max_ac_contingencies": 3,
  "participants": [
    {
      "participant_id": "firm_a",
      "participant_name": "Firm A",
      "generator_indices": [3],
      "behavior": {
        "type": "markup_and_withholding",
        "energy_markup_fraction": 0.15,
        "capacity_withholding_fraction": 0.10,
        "commitment_markup_fraction": 0.05,
        "upward_reserve_price_per_mwh": 2.0
      }
    }
  ]
}
```

The response includes model scope, cost offers, SCUC commitment and converter
directions, period dispatch, AC/DC LMP, DC voltage/branch/converter results,
hybrid nonlinear validation, preventive AC-branch N-1 cut trajectory,
full-component corrective checks, nonlinear AC contingency checks,
security-cost attribution, AC/DC and DC-storage settlement,
generator/participant settlement, behaviour audit, market-power metrics, and
the cashflow ledger. `performance` contains the model-size and stage-timing
profile, including exact network-generation candidates, activated bounds,
iterations, remaining violations, and worst violation MW; `uc_solver` accepts `auto`, `highs`, `scip`, `native`, or `gurobi`
with existing backend-availability fallback semantics.
The Case9 contracts are registered in
`tests/test_market_simulation.cpp`.

`POST /api/session/run_real_time_market` accepts the same market fields plus:

```json
{
  "realized_load_multiplier": 1.05,
  "realized_load_multipliers": [1.02, 1.04, 1.08],
  "realized_profiles": [{"id": 2, "values": [0.8, 0.7, 0.5]}],
  "real_time": {
    "run_ac_validation": false,
    "enable_n1_security": true
  },
  "ancillary_services": {
    "enabled": true,
    "generator_imbalance_tolerance_fraction": 0.02,
    "load_imbalance_tolerance_fraction": 0.02,
    "generator_imbalance_penalty_per_mwh": 50.0,
    "load_imbalance_penalty_per_mwh": 50.0,
    "reserve_performance_payment_per_mwh": 10.0,
    "reserve_nonperformance_penalty_per_mwh": 100.0,
    "reserve_performance_factor": 0.95
  }
}
```

Scalar/per-period load multipliers modify profile `0`; `realized_profiles`
can replace any load or renewable profile explicitly.  The scalar reserve
performance factor applies to every authored generator; advanced clients may
instead send `reserve_performance_factor_by_generator` in authored generator
order.

`POST /api/session/run_repeated_market_game` additionally accepts:

```json
{
  "game": {
    "max_rounds": 4,
    "markup_step_fraction": 0.05,
    "withholding_step_fraction": 0.05,
    "maximum_markup_fraction": 1.0,
    "maximum_withholding_fraction": 0.5,
    "profit_improvement_tolerance": 0.0001,
    "include_cost_based_participants": false,
    "run_real_time": true
  }
}
```

## GUI workflow

The **规划与运行 → 电力市场** module exposes the same runtime contract:

- configure the horizon, offer segmentation, reserve requirement, renewable
  curtailment cost, network constraints, DC-storage/terminal-SOC policy, and
  base-case AC validation;
- assign generators to participant IDs and choose cost-based, fixed-markup,
  capacity-withholding, or combined behaviour;
- enable preventive AC-branch N-1 cuts, full-component corrective checks,
  iteration/cut limits, emergency-rating scaling, and post-contingency AC
  certification;
- inspect nodal LMP trajectories, period clearing, DC-storage dispatch/SOC and
  settlement, participant profit/uplift, the N-1 cut trajectory, component
  checks, AC contingency failures, and the cash-flow ledger;
- apply an actual-load deviation and inspect day-ahead/real-time prices,
  dispatch deviations, two-settlement profit, and the combined cash ledger;
- enable real-time ancillary settlement, set deviation bands, penalty/reward
  prices and reserve performance, then inspect activation, delivery, shortfall,
  participant charges, and the system-operator ancillary balance;
- run bounded repeated best responses and inspect participant profit, markup,
  withholding, LMP, and HHI evolution by round;
- export either the market result alone or the combined GUI result bundle.

The browser integration contract is exercised by
`tests/e2e/market_gui_e2e.mjs` against the real HTTP server, Case9, and the
built-in `market_5bus_acdc_toy` hybrid API contract.

# Native Day-Ahead Market Runtime

The first native electricity-market vertical slice is implemented by
`hacdcpf::market::run_day_ahead_market`.  It operates on an immutable
`HybridPowerSystem` and a `TimeSeriesData` horizon:

```text
participant behaviour and submitted offers
  -> network-constrained SCUC
  -> fixed-commitment multi-period SCED
  -> iterative preventive branch N-1 cuts
  -> nodal LMP and upward-reserve price
  -> base-case and contingency AC power-flow certification
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
  limits.  SCED fixes the accepted binary decisions and retains the same
  physical dispatch envelope.
- LMPs are duals of nodal active-power balance.  The upward-reserve price is
  the dual of the system reserve-balance row.
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

## AC certification

Each commercial SCED interval is replayed through the native AC power-flow
solver after applying nodal load shedding to the physical snapshot.
`ACValidationPeriod` reports convergence, residual, physical branch loss,
voltage violation, branch thermal overload, maximum branch loading, and the
slack adjustment required to supply AC losses.  The adjusted slack output is
also checked against its active-power limits.  A converged power flow is only
`secure` when all configured voltage, thermal, and generator tolerances pass;
otherwise the market returns `ac_validation_failed`.  The commercial lossless
SCED schedule is preserved; physical slack adjustment is not silently written
back into settlement.

## Preventive N-1 security

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

DC screening treats the branch MVA rating as an active-power MW limit.  The
subsequent AC certification uses apparent power and is therefore the physical
thermal check.

## Current scope boundary

This slice supports AC buses, branches, physical generators, fixed renewable /
static generation, loads, and storage injections.  It explicitly rejects
in-service external grids and hybrid AC/DC assets rather than applying a silent
AC-only approximation.  Hybrid VSC/DC/DC-DC market co-optimisation,
probabilistic participant learning, and stochastic reserve procurement are
subsequent extensions.

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
included in resource deviation revenue and the deviation congestion-rent
identity.  `DeviationSettlementLedger::cashflow_residual` audits the combined
day-ahead and real-time cash flow.

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
  "reserve_fraction": 0.05,
  "voll_per_mwh": 10000.0,
  "exogenous_curtailment_penalty_per_mwh": 5.0,
  "network_constraints": true,
  "run_ac_validation": true,
  "enable_n1_security": true,
  "n1_max_iterations": 8,
  "n1_max_cuts_per_iteration": 200,
  "n1_max_contingencies": 0,
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

The response includes cost offers, SCUC commitment, period SCED dispatch,
upward reserve, nodal LMP, AC validation, DC N-1 cut trajectory, nonlinear AC
contingency checks, security-cost attribution, generator/participant
settlement, behaviour audit, market-power metrics, and the cashflow ledger.
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
  curtailment cost, network constraints, and base-case AC validation;
- assign generators to participant IDs and choose cost-based, fixed-markup,
  capacity-withholding, or combined behaviour;
- enable preventive N-1 cuts, iteration/cut limits, emergency-rating scaling,
  and post-contingency AC certification;
- inspect nodal LMP trajectories, period clearing, participant profit/uplift,
  the N-1 cut trajectory, AC contingency failures, and the cash-flow ledger;
- apply an actual-load deviation and inspect day-ahead/real-time prices,
  dispatch deviations, two-settlement profit, and the combined cash ledger;
- enable real-time ancillary settlement, set deviation bands, penalty/reward
  prices and reserve performance, then inspect activation, delivery, shortfall,
  participant charges, and the system-operator ancillary balance;
- run bounded repeated best responses and inspect participant profit, markup,
  withholding, LMP, and HHI evolution by round;
- export either the market result alone or the combined GUI result bundle.

The browser integration contract is exercised by
`tests/e2e/market_gui_e2e.mjs` against the real HTTP server and Case9.

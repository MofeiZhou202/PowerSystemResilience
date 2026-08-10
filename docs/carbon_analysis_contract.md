# Carbon Analysis Runtime Contract

This document covers snapshot carbon-flow tracing, annual aggregation, storage
carbon inventory, and user/node green-energy-certificate (GEC) accounting in
`carbon_analysis/`.

## Inputs and algorithms

`compute_carbon_analysis(sys, pf_result, options)` requires a converged
`PowerFlowResult` with branch and converter transfers. The convenience overload
runs power flow first. Two related calculations are returned:

- proportional tracing allocates source power and loss responsibility through
  solved flow directions;
- a sparse linear system `A w = b` solves nodal carbon intensity.

The model consumes explicit generator, external-grid, renewable, negative-load,
storage, converter, energy-router, and load semantics present in the system and
solved result. It is post-processing: it does not redispatch power or repair a
failed power flow.

## Units and identity

| Quantity | Unit or index space |
|---|---|
| Active power, demand, loss, mismatch | MW |
| Energy and GEC | MWh |
| Emissions and carbon inventory | tCO2 |
| Carbon intensity/emission factor | tCO2/MWh |
| `load_index`, `branch_index`, converter/storage/router index | Stable source component `.index` |
| `bus`, `bus_index` | Stable source bus `.index`, qualified by AC/DC field or result collection |
| `source_id` | Carbon-tracing internal ID, not a component `.index`; use `source_type` and `component_index` for attribution. |

Result vectors follow their documented source collection order and also carry
stable IDs. AC and DC loads, branches, and buses are separate collections.
`generator_supply_mw` and loss-allocation maps are keyed by carbon `source_id`.

## Validity contract

There is no single implicit success flag. Consumers must inspect:

- `power_balance_verified` and nodal mismatch diagnostics;
- `matrix_solved`, rank, relative residual, and condition estimate;
- `tracing_verified`, which requires the physical balance and allocation
  checks to pass.

If the supplied PF result is not converged, snapshot analysis returns default
non-verified output. Matrix singularity, rank deficiency, excessive condition,
or residual failure remains visible; it is not converted into a successful
zero-carbon result. `loss_allocation_alpha` changes responsibility for losses,
not physical branch loss.

## Annual and storage accounting

Annual overloads accept either one static system, one system per PF step, or a
`TimeSeriesPFResult`. `step_duration_hr` must be positive and finite. Static
system overloads are only correct when dispatch, load state, and storage state
do not change. Dynamic storage carbon requires one actual
`pf_system_snapshots[t]` for every PF result.

`step_results[t]` retains convergence and carbon validity. Non-converged steps
are counted but not treated as verified; optional hourly matrices contain NaN
at those steps. Their row is timestep and their column follows `bus_stats` or
`load_stats` order. Storage results expose initial/terminal inventory, charging,
discharging and self-discharge losses, and both energy and carbon-inventory
balance residuals.

## GEC accounting and import formats

User GEC maps stable `{load_index,is_dc}` references to an external `user_id`.
Node GEC uses stable `{bus_index,is_dc}` keys. GEC accounting reduces reported
net emissions but does not alter physical gross carbon flow. Hourly accounting
requires the corresponding hourly load arrays from annual analysis.

CSV accepts the exact documented header and exact column count. Fields may use
RFC 4180-style quotes and doubled-quote escaping. Integer, Boolean, and
non-negative floating fields must consume the complete decoded field; malformed
quotes, trailing text, NaN/Inf, duplicate keys, and conflicting user rows are
rejected with `std::invalid_argument`. JSON parsers apply the same domain and
non-negativity checks.

## Approximation boundary

Proportional sharing is an allocation convention, not a causal dispatch model.
The matrix method is an active-power steady-state model; it does not assign
reactive power, dynamic causality, or marginal emissions. Balancing-source
carbon depends on explicit external-grid/DC reference factors. Unmodelled
exports or missing sources fail the nodal-balance validity gate.

## Registered verification

- `test_carbonflow_tracing`: analytic source sharing, losses, AC/DC mappings,
  converters, storage, rich components, and invalid balances.
- `test_carbonflow_dynamic_storage`: time-series snapshots, SOC/carbon
  recurrence, balance residuals, and strict CSV parsing.
- `test_carbonflow_case_validation`: built-in and MATPOWER case validation over
  multi-step storage workflows.
- `test_crossmodule_integration`: PF/OPF/reconfiguration/carbon consistency.


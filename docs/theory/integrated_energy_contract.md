# Campus Integrated Energy Runtime Contract

This document defines `solve_campus_ies()` and the data, option, and result
types under `include/hacdcpf/integrated_energy/`.

## Model scope

The implemented model is an isolated-campus multi-carrier MILP. It balances
aggregate electricity, heat, hydrogen, fuel, transport, storage, renewable,
CHP, heat-pump, electrolyzer, fuel-cell, and CCUS flows over time.

It is not an AC network model. `pcc_ac_bus` is a stable attribution label for
the aggregate PCC exchange only. No `HybridPowerSystem`, reactive power,
voltage, branch flow, short-circuit, or security constraint enters the solve.
`fixed_power_factor` must therefore be exactly `1.0`; other values are rejected.
The result repeats this boundary in `model_scope`, `validity`, and
`model_limitations`.

## Inputs and normalization

`num_steps` and positive `step_duration_hr` define the horizon. Each profile
must have length zero, one, or `num_steps`: zero selects the field fallback and
one broadcasts the value. Power/carrier rates are MW, storage states and
energy-per-step totals are MWh, prices are per MWh, carbon is tCO2, carbon
intensity is tCO2/MWh, and transport demand is km.

Passive conversion, storage, retention, and inter-layer transfer efficiencies
must be finite and in `(0,1]`; values above unity are rejected. Two fields are
outside this check: `eta_wasteheat` and `eta_carbon_to_fuel` (both default
`0.0`) are not validated and enter the model as given — `eta_wasteheat` scales
the heat-pump waste-heat recovery term in the heat balance, and
`eta_carbon_to_fuel` only activates the synthetic-fuel bound when positive.
Heat-pump COP is a separate performance ratio and may exceed one. Current normalization clamps
negative/non-finite capacities and limits to zero, clamps storage bounds into
capacity, and restores a non-positive/non-finite heat-pump COP to `3.2`; these
legacy normalizations are not individually reported in the result and should
not be used as input-data validation.

## Carrier and result mapping

All per-step result vectors have `num_steps` elements. Storage state vectors
also expose one value per reported terminal step. Prefixes have carrier
semantics in this module:

| Prefix | Carrier/meaning |
|---|---|
| `p_` | Electric power in MW, except explicitly named electrolysis input splits. |
| `q_` | Heat rate or thermal-storage energy, not electrical reactive power. |
| `h_` | Hydrogen carrier rate/storage. |
| `f_` | Fuel carrier rate. |
| `d_` | Solved transport distance in km. |

`p_pcc_mw = p_grid_export_mw - p_grid_import_mw`; positive is campus export.
Energy totals multiply MW profiles by `step_duration_hr`. Carbon vectors are
per-step tCO2 and are summed directly. `total_transport_km` is derived from
solved EV, hydrogen-vehicle, and combustion-vehicle distances; it is zero when
transport is disabled. Sankey flows aggregate MWh by carrier and are a
visualization product, not an additional balance model.

## Options and solver outcomes

Objectives are cost, carbon, curtailment, or an explicit weighted sum. Binary
exclusivity constraints for grid exchange/storage are optional. Carbon budget,
cyclic terminal storage, transport, and weekly/seasonal hydrogen layers are
independently configurable.

`feasible` means the selected backend returned a primal solution. `optimal`
additionally requires an optimal termination and a finite gap within
`mip_gap_tol`. `solver_name`, `status`, objective, and solve time must accompany
published schedules. The AML solve permits backend fallback, so consumers must
use `solver_name` rather than infer the executed backend from the requested
enum. When no primal exists, schedule vectors remain empty.

## Approximation and unsupported behavior

- The PCC is a single active-power exchange without network losses or limits.
- Carrier balances are deterministic and aggregate; no hydraulic, thermal
  network, gas pressure, unit dynamics, or contingency model is present.
- Efficiencies are fixed linear coefficients; part-load curves and degradation
  are absent.
- Transport is aggregate distance allocation, not a traffic-network model.
- `CampusIESValidity` is capability metadata, not proof that input data are
  calibrated or that an optimal solution was found.

## Registered verification

`test_integrated_energy_campus` checks all carrier balances, storage options,
carbon-budget/CCUS behavior, honest isolated-PCC scope, invalid efficiency and
power-factor rejection, and solved transport totals.


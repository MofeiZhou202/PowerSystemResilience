# Campus Integrated Energy Simulation C++ Reimplementation Plan

Purpose: reimplement the sibling Julia `IntegratedEnergySystems` model as a native
C++ `园区综合能源仿真` module using the current MIPSolvers-based optimization
framework. The result should live inside this repository, interoperate with
`HybridPowerSystem`, and remain loosely coupled to the electrical AC/DC solvers:
the campus model owns multi-carrier dispatch; the existing C++ network stack owns
electrical deliverability.

Implementation anchors:

- `include/hacdcpf/aml/aml.hpp`: namespace bridge to `mipsolvers::aml`.
- `include/hacdcpf/engine/*`: forwarding headers to MIPSolvers engine APIs.
- `src/time_series/time_series_pf.cpp`: current large UC/MILP implementation.
- `include/hacdcpf/time_series/time_series_pf.hpp`: UC options/result patterns.
- `include/hacdcpf/model/hybrid_power_system.hpp`: existing electrical and DER data.
- `include/hacdcpf/model/{ac_components,dc_components,converter_components}.hpp`.
- Sibling Julia source: `/Users/tianyangzhao/Codes/IntegratedEnergySystems/IntegratedEnergySystems/model.jl`.

---

## 1. Scope

The C++ module should model campus-scale energy generation, conversion, storage,
transport demand, carbon accounting, and grid exchange among multiple carriers:

| Carrier | Meaning | C++ electrical interaction |
|---|---|---|
| Electricity | grid import/export, PV/wind, CHP electricity, fuel cell output, electrical load | injected into AC/DC buses |
| Heat | thermal loads, CHP heat, heat pump output, thermal storage | internal to campus optimizer |
| Hydrogen | electrolysis, industrial demand, H2 vehicles, fuel cell input, multi-timescale storage | internal, except fuel cell/electrolyzer electrical power |
| Fuel | external fuel, CHP fuel, industrial fuel, synthetic fuel, ICV refueling | internal |
| Carbon | emissions, capture, residual emissions, carbon budget | internal objective/constraints |

Out of scope for the first implementation:

- AC nonlinear power-flow equations inside the campus MILP.
- Detailed thermal/hydrogen network hydraulics.
- Stochastic scenario trees and lifecycle planning.
- Direct dependency on Julia at runtime.

---

## 2. Architecture

Add a native module under `hacdcpf/integrated_energy`.

Proposed files:

```text
include/hacdcpf/integrated_energy/
  integrated_energy_system.hpp
  integrated_energy_options.hpp
  integrated_energy_result.hpp
  integrated_energy_optimizer.hpp

src/integrated_energy/
  integrated_energy_model_builder.cpp
  integrated_energy_optimizer.cpp
  integrated_energy_result_extractor.cpp
```

Preferred layering:

```text
CampusIESData
    -> CampusIESModelBuilder using hacdcpf::aml
    -> MIPSolvers backend
    -> CampusIESResult
    -> electrical schedule applied to HybridPowerSystem snapshots
    -> existing PF/OPF validation
```

Use `hacdcpf::aml` for the first version because it is the cleanest route to
MIPSolvers and avoids another large hand-indexed MILP builder. If profiling later
shows builder overhead or extraction overhead matters, port the hot path to the
raw `engine::LinearProblem` API.

---

## 3. Data Model

Create a campus-level model separate from `HybridPowerSystem`.

Core structs:

```cpp
enum class EnergyCarrier {
  Electricity,
  Heat,
  Hydrogen,
  Fuel
};

struct CampusTimeSeries {
  int num_steps{24};
  double step_duration_hr{1.0};
};

struct CampusInterconnection {
  int pcc_ac_bus{0};
  int pcc_dc_bus{0};
  double import_limit_mw{0.0};
  double export_limit_mw{0.0};
  double fixed_power_factor{1.0};
};
```

Component groups:

```text
CampusElectricLoad
CampusThermalLoad
CampusHydrogenLoad
CampusFuelLoad
CampusRenewableUnit
CampusCHPUnit
CampusHeatPump
CampusElectrolyzer
CampusFuelCell
CampusElectricStorage
CampusThermalStorage
CampusHydrogenStorage
CampusHydrogenStorageLayer
CampusCCUSUnit
CampusTransportFleet
```

All internal quantities should use:

```text
MW, MWh, hour, tCO2, currency
```

The Julia model frequently behaves like `kW/kWh`; do unit conversion at import
boundaries and never store mixed units internally.

---

## 4. Mathematical Formulation

Let `t in T = {0,...,N-1}` and step length be `dt`.

### 4.1 Electricity Balance

For each time step:

```text
P_grid_import[t] - P_grid_export[t]
+ P_solar[t] + P_wind[t]
+ P_CHP[t] + P_fuelcell[t]
+ P_bess_dis[t] + P_EV_V2G[t]
=
P_elec_load[t]
+ P_electrolyzer_H2[t] + P_electrolyzer_fuel[t]
+ P_heatpump[t]
+ P_bess_ch[t]
+ P_EV_ch[t]
+ P_CCUS[t]
+ P_solar_curt[t] + P_wind_curt[t]
```

The net PCC schedule exported to the electrical module is:

```text
P_pcc[t] = P_grid_export[t] - P_grid_import[t]
```

Positive `P_pcc` means campus exports to the distribution network.

### 4.2 Heat Balance

```text
Q_CHP[t] + Q_HP[t] + Q_storage_dis[t] + eta_wasteheat * P_heatpump[t]
= Q_heat_load[t] + Q_storage_ch[t]
```

### 4.3 Hydrogen Balance

```text
H_electrolysis[t]
+ H_daily_dis[t] + H_weekly_dis[t] + H_seasonal_dis[t]
+ H_weekly_to_daily[t] + H_seasonal_to_weekly[t]
=
H_load[t] + H_fuelcell[t] + H_vehicle_refuel[t]
+ H_daily_ch[t] + H_weekly_ch[t] + H_seasonal_ch[t]
+ H_daily_to_weekly[t] + H_weekly_to_seasonal[t]
```

### 4.4 Fuel Balance

```text
F_CHP[t] + F_load[t] + F_vehicle_refuel[t]
= F_external[t] + F_synthetic[t]
```

### 4.5 Conversion Constraints

```text
H_electrolysis[t] = eta_electrolysis * P_electrolyzer_H2[t]
F_synthetic[t] = eta_power_to_fuel * P_electrolyzer_fuel[t]
P_fuelcell[t] = eta_fuelcell * H_fuelcell[t]
Q_HP[t] = COP_heatpump * P_heatpump[t]
Q_CHP[t] = P_CHP[t] / eta_CHP_elec * eta_CHP_th
```

For CHP fuel use, use a consistent energy accounting equation:

```text
F_CHP[t] * eta_CHP_total = P_CHP[t] + Q_CHP[t]
```

The Julia model has a slightly unusual heat-normalized term in its CHP equation.
For the C++ implementation, prefer the physically transparent total-efficiency
form above, with a compatibility option if exact Julia reproduction is needed.

### 4.6 Storage Dynamics

Generic storage:

```text
E[t+1] = eta_self * E[t]
       + eta_ch * P_ch[t] * dt
       - P_dis[t] * dt / eta_dis
```

Apply this to:

```text
electric storage
thermal storage
daily hydrogen storage
weekly hydrogen storage
seasonal hydrogen storage
EV fleet SOC
ICV/HV fleet energy state
```

Hydrogen inter-layer dynamics:

```text
H_daily[t+1] =
  eta_daily_self * H_daily[t]
  + eta_daily_ch * H_daily_ch[t] * dt
  - H_daily_dis[t] * dt / eta_daily_dis
  + eta_weekly_to_daily * H_weekly_to_daily[t] * dt
  - H_daily_to_weekly[t] * dt

H_weekly[t+1] =
  eta_weekly_self * H_weekly[t]
  + eta_weekly_ch * H_weekly_ch[t] * dt
  - H_weekly_dis[t] * dt / eta_weekly_dis
  + eta_daily_to_weekly * H_daily_to_weekly[t] * dt
  + eta_seasonal_to_weekly * H_seasonal_to_weekly[t] * dt
  - H_weekly_to_daily[t] * dt
  - H_weekly_to_seasonal[t] * dt

H_seasonal[t+1] =
  eta_seasonal_self * H_seasonal[t]
  + eta_seasonal_ch * H_seasonal_ch[t] * dt
  - H_seasonal_dis[t] * dt / eta_seasonal_dis
  + eta_weekly_to_seasonal * H_weekly_to_seasonal[t] * dt
  - H_seasonal_to_weekly[t] * dt
```

### 4.7 Carbon and CCUS

```text
Emissions[t] =
  gamma_grid[t] * P_grid_import[t] * dt
  + gamma_fuel * F_external[t] * dt

sum_t (Emissions[t] - CO2_captured[t]) <= CO2_budget

Carbon_residual[t] >= Emissions[t] - CO2_captured[t]

CO2_captured[t] <= capture_rate_fuel * F_external[t] * dt
                + capture_rate_ptf * gamma_grid[t]
                  * (P_electrolyzer_H2[t] + P_electrolyzer_fuel[t]) * dt

P_CCUS[t] >= ccus_power_per_tco2 * CO2_captured[t] / dt
```

---

## 5. MILP Features

Start with a strong LP/MILP baseline and add binaries only where needed.

Recommended first binary set:

| Feature | Binary variables | Priority |
|---|---:|---|
| Electric storage charge/discharge exclusivity | `z_bess_ch[t]`, `z_bess_dis[t]` or one mode binary | High |
| Hydrogen storage SOC regions | low/medium/high binaries | Medium |
| Grid import/export exclusivity | one import/export mode binary | Medium |
| CHP on/off with minimum stable generation | `u_chp[t]` | Medium |
| Electrolyzer on/off with minimum load | `u_el[t]` | Low |
| Fuel cell on/off with minimum load | `u_fc[t]` | Low |

Avoid porting every Julia binary immediately. First reproduce the energy balances
and dispatch economics, then add operating realism.

Use MIPSolvers AML bridge utilities where useful:

```text
abs_value_bridge      -> storage throughput / degradation cost
indicator_bridge      -> on/off logic
piecewise_linear      -> part-load efficiency, carbon price blocks
max_epigraph_bridge   -> residual penalties
```

---

## 6. Objective Function

Add configurable objective modes:

```cpp
enum class CampusIESObjective {
  Cost,
  Carbon,
  MinCurtailment,
  Weighted
};
```

Cost objective:

```text
min sum_t dt * (
  c_grid_buy[t] * P_grid_import[t]
- c_grid_sell[t] * P_grid_export[t]
+ c_fuel * F_external[t]
+ c_solar * P_solar[t]
+ c_wind * P_wind[t]
+ c_chp_elec * P_CHP[t]
+ c_chp_heat * Q_CHP[t]
+ c_heatpump * P_heatpump[t]
+ c_electrolyzer * (P_electrolyzer_H2[t] + P_electrolyzer_fuel[t])
+ c_fuelcell * P_fuelcell[t]
+ c_storage_throughput * throughput[t]
+ c_hydrogen_transfer * transfer[t]
+ c_curt_solar * P_solar_curt[t]
+ c_curt_wind * P_wind_curt[t]
+ c_ccus_capture * CO2_captured[t]
+ c_ccus_power * P_CCUS[t]
+ c_carbon * Carbon_residual[t]
+ c_dr * demand_response_shift[t]
)
+ annualized_capex_terms
```

Weighted objective:

```text
min w_cost * cost
  + w_carbon * net_emissions
  + w_curtailment * renewable_curtailment
  + w_grid * grid_import
```

---

## 7. MIPSolvers Integration

### 7.1 Preferred Builder Path

Use `hacdcpf::aml`:

```cpp
#include "hacdcpf/aml/aml.hpp"

namespace hacdcpf::integrated_energy {

CampusIESResult solve_campus_ies(const CampusIESData& data,
                                 const CampusIESOptions& options);

}
```

The builder should:

1. Create ordered set `T`.
2. Create component sets for storage, renewables, CHP, heat pumps, electrolyzers,
   fuel cells, fleets, and CCUS units.
3. Register parameters from `CampusIESData`.
4. Add variables with explicit lower and upper bounds.
5. Add balance, conversion, storage, capacity, carbon, and optional MILP constraints.
6. Add objective.
7. Solve through MIPSolvers backend.
8. Extract `CampusIESResult`.

### 7.2 Solver Options

```cpp
enum class CampusIESSolver {
  Auto,
  HiGHS,
  SCIP,
  Gurobi,
  Native
};

struct CampusIESOptions {
  CampusIESSolver solver{CampusIESSolver::Auto};
  CampusIESObjective objective{CampusIESObjective::Cost};
  bool enable_storage_exclusivity{true};
  bool enable_chp_commitment{false};
  bool enable_electrolyzer_commitment{false};
  bool enable_fuelcell_commitment{false};
  bool enable_hydrogen_soc_regions{false};
  bool enforce_terminal_storage_cyclic{false};
  bool verbose{false};
  double mip_gap{1e-4};
  int threads{0};
};
```

Auto backend order:

```text
HiGHS -> SCIP -> Gurobi if explicitly enabled -> Native fallback
```

This should match the current project preference for portable in-tree solvers,
while keeping commercial solvers opt-in.

---

## 8. Electrical Coupling

The campus optimizer should not solve network physics. It should produce an
electrical schedule that can be validated by the existing AC/DC stack.

Two coupling modes:

### 8.1 PCC Aggregate Mode

The campus is represented as one net injection at a point of common coupling:

```text
P_pcc[t] = P_grid_export[t] - P_grid_import[t]
Q_pcc[t] = P_pcc[t] * tan(acos(power_factor))
```

Mapping:

```text
P_pcc > 0 -> StaticGenerator at PCC
P_pcc < 0 -> Load at PCC
```

### 8.2 Component Bus Mode

Each electrical component has a target AC/DC bus:

```text
PV/wind/fuel cell/CHP          -> generator/static generator
electrolyzer/heat pump/CCUS    -> load
electric storage               -> Storage p_mw = discharge - charge
EV charging/V2G                -> ChargingStation or Storage-like schedule
```

This mode enables local voltage and feeder-congestion validation.

Recommended first implementation: PCC aggregate mode. Add component-bus mode
after result extraction and validation are stable.

---

## 9. Public API

Suggested API:

```cpp
namespace hacdcpf::integrated_energy {

CampusIESResult solve_campus_ies(const CampusIESData& data,
                                 const CampusIESOptions& options = {});

std::vector<HybridPowerSystem> apply_campus_schedule(
    const HybridPowerSystem& base,
    const CampusIESData& data,
    const CampusIESResult& result);

CampusIESNetworkValidationResult validate_campus_schedule(
    const HybridPowerSystem& base,
    const CampusIESData& data,
    const CampusIESResult& result,
    const PowerFlowOptions& pf_options);

}
```

`CampusIESResult` should contain:

```text
feasible
solver_name
termination_status
objective
total_cost
total_emissions
total_curtailment_mwh
p_pcc_mw[t]
component schedules
storage state trajectories
hydrogen layer trajectories
carbon trajectories
warnings
```

---

## 10. JSON I/O

Add JSON serialization after the core structs stabilize:

```text
src/io/integrated_energy_json.cpp
include/hacdcpf/io/integrated_energy_json.hpp
```

Use the same `nlohmann_json` style as `src/io/json_io.cpp`.

Minimal input schema:

```json
{
  "num_steps": 24,
  "step_duration_hr": 1.0,
  "interconnection": {
    "pcc_ac_bus": 1,
    "import_limit_mw": 10.0,
    "export_limit_mw": 5.0
  },
  "profiles": {
    "electric_load_mw": [],
    "heat_load_mw": [],
    "hydrogen_load_mw": [],
    "solar_available_mw": [],
    "wind_available_mw": [],
    "grid_buy_price": [],
    "grid_carbon_tco2_mwh": []
  },
  "components": {
    "heat_pumps": [],
    "electrolyzers": [],
    "fuel_cells": [],
    "chp_units": [],
    "storages": [],
    "ccus_units": []
  }
}
```

---

## 11. Validation Strategy

### 11.1 Unit Tests

Add focused tests:

```text
tests/test_integrated_energy_basic.cpp
tests/test_integrated_energy_storage.cpp
tests/test_integrated_energy_hydrogen_layers.cpp
tests/test_integrated_energy_carbon.cpp
tests/test_integrated_energy_grid_coupling.cpp
tests/test_integrated_energy_json.cpp
```

Test cases:

1. Electricity-only campus reduces to grid import/load balance.
2. Heat pump satisfies heat demand and increases electrical demand.
3. Electrolyzer plus fuel cell round trip respects efficiencies.
4. Electric storage shifts energy from low-price to high-price hours.
5. Hydrogen daily/weekly/seasonal storage obeys capacity and transfer limits.
6. Carbon budget forces fuel reduction, CCUS, or renewable use.
7. PCC schedule applied to `HybridPowerSystem` passes power-flow validation.

### 11.2 Julia Cross-Validation

Before extending beyond the Julia model, build one 24-hour benchmark:

```text
same load profiles
same renewable profiles
same efficiencies
same storage bounds
same carbon prices/budget
same grid prices
```

Compare:

```text
objective
P_grid
P_heatpump
P_electrolyzer_H2
P_CHP
P_fuelcell
storage state trajectories
hydrogen layer trajectories
emissions and CO2_captured
energy-balance residuals
```

Use tolerances rather than exact equality because solver choices and cleaned CHP
equations may differ.

---

## 12. Implementation Phases

### Phase A: Skeleton and LP Baseline

- Add headers and source files.
- Add `CampusIESData`, options, and result structs.
- Implement electricity-only balance with grid import/export and fixed load.
- Solve through `hacdcpf::aml`.
- Add first unit test.

Deliverable: native C++ optimizer can solve a simple campus import/export problem.

### Phase B: Core Multi-Carrier Model

- Add heat pump, CHP, electrolyzer, fuel cell.
- Add electricity, heat, hydrogen, and fuel balances.
- Add constant-efficiency conversion equations.
- Add renewable availability and curtailment.
- Add cost objective.

Deliverable: deterministic LP equivalent of the core Julia dispatch model.

### Phase C: Storage and Transport

- Add electric, thermal, and hydrogen storage dynamics.
- Add daily/weekly/seasonal hydrogen layers.
- Add EV/HV/ICV fleet energy dynamics.
- Add optional terminal cyclic constraints.

Deliverable: campus model can simulate temporal flexibility.

### Phase D: Carbon and CCUS

- Add emissions calculation.
- Add carbon budget.
- Add carbon residual penalty.
- Add CCUS capture and energy-use constraints.

Deliverable: low-carbon campus dispatch.

### Phase E: MILP Operational Realism

- Add charge/discharge exclusivity.
- Add CHP commitment if needed.
- Add hydrogen SOC region binaries.
- Add optional grid import/export exclusivity.
- Add piecewise linear efficiency or cost curves.

Deliverable: full campus MILP using MIPSolvers.

### Phase F: Electrical Coupling

- Implement PCC aggregate schedule application.
- Run existing PF validation per time step.
- Report line/voltage violations and infeasible electrical schedules.
- Add component-bus mapping after PCC mode is stable.

Deliverable: `园区综合能源仿真` integrated with AC/DC distribution analysis.

### Phase G: JSON and UI/API Integration

- Add JSON import/export.
- Add API entry points.
- Add GUI or server endpoint only after core API and tests are stable.

Deliverable: user-facing campus IES simulation workflow.

---

## 13. Recommended First PR Boundary

Keep the first PR deliberately narrow:

```text
include/hacdcpf/integrated_energy/*
src/integrated_energy/*
tests/test_integrated_energy_basic.cpp
tests/test_integrated_energy_storage.cpp
CMakeLists.txt
tests/CMakeLists.txt
```

First PR feature set:

- Electricity balance.
- Renewable availability.
- Grid import/export.
- Electric storage.
- Cost objective.
- MIPSolvers AML backend.
- No JSON, no GUI, no network validation yet.

This creates the foundation without mixing in every carrier at once.

---

## 14. Key Design Decisions

1. Use `hacdcpf::aml` first, raw `engine` later only if needed.
2. Keep campus multi-carrier physics separate from `HybridPowerSystem`.
3. Use MW/MWh internally.
4. Export only electrical injections to the power-flow module.
5. Add binaries incrementally.
6. Validate against Julia for a small benchmark, then let the C++ version evolve.
7. Make solver backend configurable and avoid hard-coding Gurobi.

---

## 15. Open Questions

- Should the first C++ model exactly reproduce Julia's CHP equation, or use the
  physically clearer total-efficiency equation?
- Should campus electrical coupling initially be PCC-only, or should component
  bus mapping be required from the beginning?
- Do hydrogen quantities represent lower heating value energy, mass converted to
  energy, or abstract energy-equivalent units? The C++ model should pick one and
  document it.
- Should carbon use `kgCO2` or `tCO2` internally? This plan recommends `tCO2`.
- Should annualized investment cost be fixed from input capacities, or should
  later phases include capacity expansion decision variables?


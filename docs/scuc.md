# SCUC/SCED/LMP Module — Reference Documentation

The **SCUC module** (`mipsolvers::scuc`) provides a standalone Security-Constrained
Unit Commitment (SCUC) solver with a Security-Constrained Economic Dispatch (SCED)
re-dispatch stage and a Locational Marginal Price (LMP) computation stage.

Inputs and outputs are exchanged via JSON.  The solver backend is any adapter
registered with `mipsolvers::engine::SolverEngine`, chosen at runtime via the
`config.solver` field in the input JSON.

---

## Table of Contents

1. [Architecture Overview](#1-architecture-overview)
2. [Quick Start](#2-quick-start)
3. [C++ API](#3-c-api)
4. [Input Data Types](#4-input-data-types)
   - [BidSegment](#bidSegment)
   - [Generator](#generator)
   - [Branch](#branch)
   - [Load](#load)
   - [WindUnit / SolarUnit](#windunit--solarunit)
   - [StorageUnit](#storageunit)
   - [DCLine](#dcline)
   - [SCUCConfig](#scucconfig)
   - [SCUCProfiles](#scucprofiles)
   - [SCUCInitialStatus](#scucinitialstatus)
   - [SCUCInput](#scucinput)
5. [Output Data Types](#5-output-data-types)
   - [SCUCSolveResult](#scucsolvresult)
   - [SCUCLMPResult](#scuclmpresult)
   - [SCUCOutput](#scucoutput)
6. [JSON Input Schema](#6-json-input-schema)
7. [JSON Output Schema](#7-json-output-schema)
8. [Mathematical Formulation](#8-mathematical-formulation)
   - [Variable Index](#81-variable-index)
   - [Objective](#82-objective)
   - [Generator Constraints](#83-generator-constraints)
   - [Storage Constraints](#84-storage-constraints)
   - [Transmission Constraints](#85-transmission-constraints)
   - [Power Balance](#86-power-balance)
   - [Reserve Requirements](#87-reserve-requirements)
   - [Market Cutting Planes](#88-market-cutting-planes)
9. [SCED Stage](#9-sced-stage)
10. [LMP Computation](#10-lmp-computation)
11. [Solver Selection](#11-solver-selection)
12. [CLI Tool](#12-cli-tool)
13. [Build Integration](#13-build-integration)

---

## 1. Architecture Overview

```
          ┌─────────────────────────────────────────────────────┐
          │                  scuc_solve(input)                  │
          └──────────────────────┬──────────────────────────────┘
                                 │
          ┌──────────────────────▼──────────────────────────────┐
          │  Stage 1: SCUC (MILP)                               │
          │  • Build MILP formulation (build_formulation)       │
          │  • Solve with SolverEngine::solve_milp              │
          │  • Extract commitment schedule + cost               │
          └──────────────────────┬──────────────────────────────┘
                                 │  if config.solve_sced
          ┌──────────────────────▼──────────────────────────────┐
          │  Stage 2: SCED (LP)                                 │
          │  • Fix binary commitment variables from Stage 1     │
          │  • Re-solve LP (SolverEngine::solve_lp)             │
          │  • Extract refined dispatch + cost                  │
          └──────────────────────┬──────────────────────────────┘
                                 │  if config.solve_lmp
          ┌──────────────────────▼──────────────────────────────┐
          │  Stage 3: LMP (LP + duals)                         │
          │  • Build delta-neighbourhood LP                     │
          │  • Solve with dual-returning solver                 │
          │  • Extract power-balance duals → nodal LMPs        │
          └─────────────────────────────────────────────────────┘
```

---

## 2. Quick Start

### C++ API

```cpp
#include "mipsolvers/scuc/scuc.hpp"

using namespace mipsolvers::scuc;

// 1. Parse input from JSON file
std::string json = read_file("day_ahead.json");
SCUCInput input = scuc_from_json(json);

// 2. Solve
SCUCOutput output = scuc_solve(input);

// 3. Write results
if (output.scuc.converged)
    std::cout << "SCUC objective: " << output.scuc.objective << "\n";
if (output.lmp.converged)
    std::cout << "Avg LMP: " << output.lmp.avg_lmp << " $/MWh\n";

// 4. Serialise to JSON
std::string result_json = scuc_output_to_json(output, input, /*indent=*/2);
```

### CLI

```bash
# Auto solver selection
scuc_solve input.json output.json

# Force HiGHS; skip LMP stage
scuc_solve input.json output.json --solver HiGHS --no-lmp

# Print to stdout with compact JSON
scuc_solve input.json --indent 0
```

---

## 3. C++ API

All symbols are in namespace `mipsolvers::scuc`.

```cpp
// Include path
#include "mipsolvers/scuc/scuc.hpp"

namespace mipsolvers::scuc {

/// Parse SCUCInput from a JSON string.
SCUCInput scuc_from_json(const std::string& json_str);

/// Solve the full SCUC → SCED → LMP pipeline.
SCUCOutput scuc_solve(const SCUCInput& input);

/// Serialise SCUCOutput to a JSON string.
/// indent = 0 for compact, 2 for human-readable
std::string scuc_output_to_json(const SCUCOutput& output,
                                const SCUCInput&  input,
                                int indent = 2);

} // namespace mipsolvers::scuc
```

---

## 4. Input Data Types

### `BidSegment`

One step of a piecewise-linear generator cost curve.

```cpp
struct BidSegment {
  double price{0.0};     // Marginal cost ($/MWh)
  double quantity{0.0};  // Capacity in this segment (MW above Pmin)
};
```

The total bid curve for a generator with $n$ segments is:

$$C_g(p) = c_{g,0}^{nl} + \sum_{k=1}^{n} \lambda_{g,k} \cdot q_{g,k}$$

where $\lambda_{g,k}$ is the segment price and $q_{g,k}$ is the segment quantity.
$p = P_{g,\min} + \sum_k q_{g,k}$.

### `Generator`

Thermal or dispatchable generator.

| Field | Type | Default | Description |
|---|---|---|---|
| `name` | `string` | `""` | Identifier |
| `bus` | `int` | 0 | 0-based bus index |
| `pmin` | `double` | 0.0 | Minimum stable output (MW) |
| `pmax` | `double` | 0.0 | Maximum output (MW) |
| `ramp_up_mw_min` | `double` | 0.0 | Ramp-up rate (MW/minute) |
| `ramp_dn_mw_min` | `double` | 0.0 | Ramp-down rate (MW/minute) |
| `min_up_time_hr` | `double` | 0.0 | Minimum on-time before shutdown (hours) |
| `min_dn_time_hr` | `double` | 0.0 | Minimum off-time before restart (hours) |
| `must_run` | `bool` | `false` | Force $u_{g,t} = 1$ for all $t$ |
| `max_startups` | `int` | 0 | Maximum startups in horizon (0 = unlimited) |
| `max_shutdowns` | `int` | 0 | Maximum shutdowns in horizon (0 = unlimited) |
| `bid_segments` | `vector<BidSegment>` | `[]` | Cost curve segments |
| `startup_cost` | `double` | 0.0 | One-time startup cost ($) |
| `no_load_cost` | `double` | 0.0 | No-load cost ($/hour while on) |
| `spinning_reserve_price` | `double` | 0.0 | Reserve bid ($/MWh) |
| `regulation_up_price` | `double` | 0.0 | Reg-up bid ($/MWh) |
| `regulation_down_price` | `double` | 0.0 | Reg-down bid ($/MWh) |
| `pfr_alpha` | `double` | 0.0 | Primary frequency regulation coefficient: max PFR = `pfr_alpha × pmax` (0 = no PFR) |

### `Branch`

AC transmission branch (line or transformer), modelled as a lossless DC
branch with reactance $x$.

| Field | Type | Default | Description |
|---|---|---|---|
| `from` | `int` | 0 | Sending-end bus (0-based) |
| `to` | `int` | 0 | Receiving-end bus (0-based) |
| `reactance` | `double` | 0.05 | Series reactance (pu) |
| `rating_mw` | `double` | 1e6 | Thermal rating (MW) |
| `in_service` | `bool` | `true` | Include in PTDF computation |

### `Load`

Static demand at a bus, scaled by a time-series profile.

| Field | Type | Description |
|---|---|---|
| `bus` | `int` | 0-based bus index |
| `p_mw` | `double` | Base active demand (MW) |

Effective demand at period $t$: $D_{d,t} = p^{\text{mw}}_d \cdot \text{profile}[d][t]$.

### `WindUnit` / `SolarUnit`

Non-dispatchable renewable units.

| Field | Type | Description |
|---|---|---|
| `bus` | `int` | 0-based bus |
| `pmax` | `double` | Rated capacity (MW) |

Forecast output at period $t$ is given by `profiles.wind[w][t]` (absolute MW)
for wind, and `profiles.solar[s][t]` for solar.  When the profile array is
absent or shorter than $T$, `pmax` is used as the forecast for missing periods.

### `StorageUnit`

Battery or pumped-hydro storage unit.

| Field | Type | Default | Description |
|---|---|---|---|
| `bus` | `int` | 0 | 0-based bus |
| `pmax_charge` | `double` | 0.0 | Max charge rate (MW) |
| `pmax_discharge` | `double` | 0.0 | Max discharge rate (MW) |
| `energy_capacity_mwh` | `double` | 0.0 | Usable energy capacity (MWh) |
| `efficiency` | `double` | 0.9 | Round-trip efficiency ($\eta_{rt}$, 0–1) |
| `soc_init` | `double` | 0.5 | Initial SoC (fraction of capacity) |
| `charge_bid_price` | `double` | 0.0 | Charge bid ($/MWh) |
| `discharge_bid_price` | `double` | 0.0 | Discharge bid ($/MWh) |
| `cycle_limit` | `double` | 0.0 | Max daily equivalent full cycles (0 = none) |

### `DCLine`

Controllable VSC-HVDC link.

| Field | Type | Default | Description |
|---|---|---|---|
| `from` | `int` | 0 | Injection bus |
| `to` | `int` | 0 | Withdrawal bus |
| `pmin` | `double` | -1e6 | Min power flow (MW, negative = reverse) |
| `pmax` | `double` | 1e6 | Max power flow (MW) |
| `ramp_up` | `double` | 1e6 | Ramp-up limit (MW/period) |
| `ramp_dn` | `double` | 1e6 | Ramp-down limit (MW/period) |

### `SCUCConfig`

Solver and model configuration.

| Field | Type | Default | Description |
|---|---|---|---|
| `solver` | `string` | `"Auto"` | Solver name: `"Auto"`, `"StrictHiGHS"`, `"Gurobi"`, `"HiGHS"`, `"SCIP"`, `"NativeBranchAndCut"` |
| `allow_fallback` | `bool` | `true` | Fall back to next solver on failure |
| `num_periods` | `int` | 24 | Dispatch time periods $T$ |
| `period_length_hr` | `double` | 1.0 | Hours per period $\Delta t$ |
| `n_segments` | `int` | 3 | Bid curve segments per generator |
| `mip_gap` | `double` | 0.001 | SCUC relative MIP gap tolerance |
| `time_limit_sec` | `double` | 300.0 | SCUC wall-clock time limit |
| `verbose` | `bool` | `false` | Print solver output |
| `spinning_reserve_req` | `double` | 0.10 | Spinning reserve requirement (fraction of load) |
| `regulation_up_req` | `double` | 0.05 | Regulation-up requirement (fraction) |
| `regulation_down_req` | `double` | 0.05 | Regulation-down requirement (fraction) |
| `voll` | `double` | 10 000.0 | Value of lost load ($/MWh) |
| `vocc` | `double` | 1 000.0 | Value of over-generation curtailment ($/MWh) |
| `renewable_min_output_coeff` | `double` | 0.0 | Minimum renewable output: $\alpha \cdot \text{forecast}$ |
| `enable_market_cuts` | `bool` | `true` | Add LP-valid pre-formulation cutting planes |
| `M1_line_slack_penalty` | `double` | 1e5 | Big-M penalty for line flow slack variables |
| `solve_sced` | `bool` | `true` | Run SCED LP after SCUC |
| `solve_lmp` | `bool` | `true` | Compute LMPs after SCED |
| `lmp_delta` | `double` | 0.10 | Delta-neighbourhood factor for LMP re-dispatch |
| `neg_reserve_req` | `double` | 0.0 | Negative (downward) reserve requirement (fraction of load); 0 = disabled |
| `pfr_reserve_req_mw` | `double` | 0.0 | Primary frequency regulation requirement (MW); 0 = disabled |

### `SCUCProfiles`

Time-series data indexed by `[unit][period]`.

```cpp
struct SCUCProfiles {
  vector<vector<double>> load;   // [nd][T]  per-unit multipliers
  vector<vector<double>> wind;   // [nw][T]  absolute MW forecast
  vector<vector<double>> solar;  // [npv][T] absolute MW forecast
};
```

- `load[d][t]` is the per-unit factor applied to `loads[d].p_mw` at period $t$.
- `wind[w][t]` and `solar[s][t]` are absolute MW forecasts (not per-unit).

Missing profile rows or entries default to: load → 1.0, wind/solar → `pmax`.

### `SCUCInitialStatus`

Generator and storage state at the beginning of the planning horizon (before
period 0).

```cpp
struct SCUCInitialStatus {
  vector<double> commitment;   // [ng]       0 or 1
  vector<double> dispatch;     // [ng]       MW dispatched in period -1
  vector<double> storage_soc;  // [nstorage] fraction of capacity
};
```

### `SCUCInput`

Aggregated problem input.

```cpp
struct SCUCInput {
  SCUCConfig       config;
  int              num_buses{1};
  vector<Generator>   generators;
  vector<Branch>      branches;
  vector<Load>        loads;
  vector<WindUnit>    wind;
  vector<SolarUnit>   solar;
  vector<StorageUnit> storage;
  vector<DCLine>      dc_lines;
  SCUCProfiles        profiles;
  SCUCInitialStatus   initial_status;
};
```

`num_buses` may be set to 0 and will then be auto-inferred as
`max(generator bus, load bus) + 1`.

---

## 5. Output Data Types

### `SCUCSolveResult`

Result from one MILP or LP stage (SCUC or SCED).

| Field | Type | Description |
|---|---|---|
| `converged` | `bool` | `true` if the solver found a feasible solution within the gap tolerance |
| `objective` | `double` | Optimal objective value ($) |
| `solver_name` | `string` | Name of the adapter that solved the problem |
| `solve_time_sec` | `double` | Wall-clock time for this stage |
| `mip_gap` | `double` | Final relative MIP gap (MILP only) |
| `n_cuts_added` | `int` | Pre-formulation market cuts added |
| `commitment` | `Matrix2D [ng][T_commit]` | Binary commitment status $u_{g,h}$ |
| `startup` | `Matrix2D [ng][T_commit]` | Binary startup indicator $v_{g,h}$ |
| `shutdown` | `Matrix2D [ng][T_commit]` | Binary shutdown indicator $w_{g,h}$ |
| `dispatch` | `Matrix2D [ng][T]` | Total generator output $p_{g,t}$ (MW) |
| `spinning_reserve` | `Matrix2D [ng][T]` | Spinning reserve $r_{g,t}^{sp}$ (MW) |
| `regulation_up` | `Matrix2D [ng][T]` | Regulation-up $r_{g,t}^{up}$ (MW) |
| `regulation_down` | `Matrix2D [ng][T]` | Regulation-down $r_{g,t}^{dn}$ (MW) |
| `segment_dispatch` | `vector<Matrix2D> [K][ng][T]` | Per-segment dispatch |
| `wind_generation` | `Matrix2D [nw][T]` | Wind output $p_{w,t}^W$ (MW) |
| `solar_generation` | `Matrix2D [npv][T]` | Solar output $p_{s,t}^{PV}$ (MW) |
| `storage_charging` | `Matrix2D [nstorage][T]` | Storage charge rate (MW) |
| `storage_discharging` | `Matrix2D [nstorage][T]` | Storage discharge rate (MW) |
| `storage_soc` | `Matrix2D [nstorage][T]` | State of charge (MWh) |
| `line_flows` | `Matrix2D [nl][T]` | Branch power flow $f_{l,t}$ (MW) |
| `load_shedding` | `vector<double> [T]` | System load shed (MW) |
| `gen_curtailment` | `vector<double> [T]` | Renewable curtailment (MW) |
| `energy_cost` | `double` | Segment bid energy cost ($) |
| `startup_cost` | `double` | Startup cost ($) |
| `no_load_cost` | `double` | No-load cost ($) |
| `reserve_cost` | `double` | Reserve bid cost ($) |
| `penalty_cost` | `double` | Load-shed + curtailment penalty ($) |
| `total_cost` | `double` | Sum of all cost components ($) |

`T_commit` = $T / \text{intervals\_per\_hour}$. For $\Delta t = 1$ h,
$T_\text{commit} = T$.

`Matrix2D` is `std::vector<std::vector<double>>`.

### `SCUCLMPResult`

| Field | Type | Description |
|---|---|---|
| `converged` | `bool` | `true` if the LMP LP converged |
| `solve_time_sec` | `double` | LMP LP solve time |
| `nodal_lmp` | `Matrix2D [nb][T]` | Nodal LMP ($/MWh) |
| `energy_lmp` | `Matrix2D [nb][T]` | Energy component of LMP |
| `congestion_lmp` | `Matrix2D [nb][T]` | Congestion component of LMP |
| `avg_lmp` | `double` | Load-weighted average system LMP |
| `max_lmp` | `double` | Maximum nodal LMP over all buses and periods |
| `min_lmp` | `double` | Minimum nodal LMP |

### `SCUCOutput`

```cpp
struct SCUCOutput {
  SCUCSolveResult scuc;  // Stage 1: SCUC MILP result
  SCUCSolveResult sced;  // Stage 2: SCED LP result (empty if solve_sced=false)
  SCUCLMPResult   lmp;   // Stage 3: LMP result   (empty if solve_lmp=false)
};
```

---

## 6. JSON Input Schema

All fields are optional unless marked **required**.

```json
{
  "config": {
    "solver":               "Auto",
    "allow_fallback":       true,
    "num_periods":          24,
    "period_length_hr":     1.0,
    "n_segments":           3,
    "mip_gap":              0.001,
    "time_limit_sec":       300,
    "verbose":              false,
    "spinning_reserve_req": 0.10,
    "regulation_up_req":    0.05,
    "regulation_down_req":  0.05,
    "voll":                 10000.0,
    "vocc":                 1000.0,
    "renewable_min_output_coeff": 0.0,
    "enable_market_cuts":   true,
    "M1_line_slack_penalty": 100000.0,
    "solve_sced":           true,
    "solve_lmp":            true,
    "lmp_delta":            0.10
  },
  "num_buses": 5,
  "generators": [                   // REQUIRED: at least one
    {
      "name":   "G1",
      "bus":    0,
      "pmin":   50.0,
      "pmax":   200.0,
      "ramp_up_mw_min": 5.0,
      "ramp_dn_mw_min": 5.0,
      "min_up_time_hr": 4.0,
      "min_dn_time_hr": 2.0,
      "must_run": false,
      "max_startups":  0,
      "max_shutdowns": 0,
      "bid_segments": [
        {"price": 25.0, "quantity": 100.0},
        {"price": 35.0, "quantity":  80.0},
        {"price": 50.0, "quantity":  20.0}
      ],
      "startup_cost":  500.0,
      "no_load_cost":  120.0,
      "spinning_reserve_price": 4.0,
      "regulation_up_price":    6.0,
      "regulation_down_price":  5.0
    }
  ],
  "branches": [                     // optional; omit for copper-plate
    {
      "from":       0,
      "to":         1,
      "reactance":  0.1,
      "rating_mw":  200.0,
      "in_service": true
    }
  ],
  "loads": [                        // REQUIRED
    {"bus": 0, "p_mw": 300.0},
    {"bus": 1, "p_mw": 150.0}
  ],
  "wind":    [],                    // optional
  "solar":   [],                    // optional
  "storage": [],                    // optional
  "dc_lines": [],                   // optional
  "profiles": {
    "load":  [[0.9, 1.0, 0.85]],   // [nloads][T] per-unit factors
    "wind":  [],
    "solar": []
  },
  "initial_status": {
    "commitment":  [1.0, 0.0],     // [ng]
    "dispatch":    [180.0, 0.0],   // [ng] MW in period -1
    "storage_soc": []              // [nstorage] fraction of capacity
  }
}
```

### StorageUnit JSON fields

```json
{
  "bus": 2,
  "pmax_charge":        50.0,
  "pmax_discharge":     50.0,
  "energy_capacity_mwh": 200.0,
  "efficiency":         0.90,
  "soc_init":           0.50,
  "charge_bid_price":   5.0,
  "discharge_bid_price": 10.0,
  "cycle_limit":        1.0
}
```

### DCLine JSON fields

```json
{
  "from":    0,
  "to":      3,
  "pmin":   -100.0,
  "pmax":    100.0,
  "ramp_up": 50.0,
  "ramp_dn": 50.0
}
```

---

## 7. JSON Output Schema

```json
{
  "meta": {
    "solver":       "Gurobi",
    "num_periods":  24,
    "num_buses":    5,
    "num_generators": 3,
    "solve_sced":   true,
    "solve_lmp":    true
  },
  "scuc": {
    "converged":     true,
    "objective":     245780.50,
    "solver_name":   "Gurobi",
    "solve_time_sec": 4.32,
    "mip_gap":       0.00082,
    "n_cuts_added":  12,
    "commitment":    [[1,1,1,...], [0,1,1,...]],  // [ng][T_commit]
    "startup":       [[0,0,0,...], [0,1,0,...]],
    "shutdown":      [[0,0,0,...], [0,0,0,...]],
    "dispatch":      [[180,200,190,...], [0,80,100,...]],
    "spinning_reserve": [[20,0,10,...], [0,20,0,...]],
    "regulation_up":    [...],
    "regulation_down":  [...],
    "wind_generation":  [...],
    "solar_generation": [...],
    "storage_charging": [...],
    "storage_discharging": [...],
    "storage_soc":      [...],
    "line_flows":       [[45,-20,...], ...],
    "load_shedding":    [0,0,0,...],
    "gen_curtailment":  [0,0,0,...],
    "cost": {
      "energy":   195000.0,
      "startup":   12000.0,
      "no_load":   18000.0,
      "reserve":    1500.0,
      "penalty":       0.0,
      "total":    226500.0
    }
  },
  "sced": { /* same structure as scuc */ },
  "lmp": {
    "converged":    true,
    "solve_time_sec": 0.41,
    "avg_lmp":      38.50,
    "max_lmp":      55.00,
    "min_lmp":      25.00,
    "nodal":        [[35,40,35,...], [38,42,37,...]],  // [nb][T]
    "energy":       [[35,40,35,...], ...],
    "congestion":   [[0,0,0,...], [3,2,2,...]]
  }
}
```

---

## 8. Mathematical Formulation

### 8.1 Variable Index

The MILP has a flat variable vector $x \in \mathbb{R}^{n_x}$.
Blocks are allocated sequentially:

| Block | Size | Description |
|---|---|---|
| $v$ (SU) | $n_g \times T_c$ | Startup binary $v_{g,h}$ |
| $w$ (SD) | $n_g \times T_c$ | Shutdown binary $w_{g,h}$ |
| $u$ (IG) | $n_g \times T_c$ | Commitment binary $u_{g,h}$ |
| $q_k$ (SEG_k) | $n_g \times T$ per $k$ | Segment dispatch (MW above Pmin) |
| $p$ (PG) | $n_g \times T$ | Total dispatch $p_{g,t}$ |
| $r^{sp}$ | $n_g \times T$ | Spinning reserve |
| $r^{up}$ | $n_g \times T$ | Regulation up |
| $r^{dn}$ | $n_g \times T$ | Regulation down |
| $f$ (PF) | $n_l \times T$ | Line power flow $f_{l,t}$ |
| $p^W$ (PW) | $n_w \times T$ | Wind generation |
| $p^{PV}$ (PPV) | $n_{pv} \times T$ | Solar generation |
| $p^{ch}$ | $n_s \times T$ | Storage charge rate |
| $p^{dis}$ | $n_s \times T$ | Storage discharge rate |
| $E$ (SOC) | $n_s \times T$ | State of charge (MWh) |
| $p^{DC}$ | $n_{dc} \times T$ | DC line power |
| $s^+_L, s^-_L$ | $T$ each | Load shed / over-generation slack |
| $s^+_l, s^-_l$ | $n_l \times T$ each | Line flow positive / negative slack |

$T_c = T / \text{intervals\_per\_hour}$.  For hourly resolution, $T_c = T$.

### 8.2 Objective

$$\min \sum_{g,t} \left( \sum_k \lambda_{g,k} q_{g,k,t} + c_g^{sp} r_{g,t}^{sp} + c_g^{up} r_{g,t}^{up} + c_g^{dn} r_{g,t}^{dn} \right) \Delta t + \sum_{g,h} \left( C_g^{SU} v_{g,h} + C_g^{NL} u_{g,h} / \text{iph} \right) + \text{VOLL} \cdot \sum_t s_t^+ \cdot \Delta t + \text{VOCC} \cdot \sum_t s_t^- \cdot \Delta t + M_1 \cdot \sum_{l,t} (s_{l,t}^{+} + s_{l,t}^{-}) \cdot \Delta t$$

where $\lambda_{g,k}$ is the bid price of segment $k$, $\Delta t$ is the period
length in hours, $C_g^{SU}$ is the startup cost, and $C_g^{NL}$ is the no-load
cost per hour.

### 8.3 Generator Constraints

**Dispatch definition** (equality for each $g,t$):
$$p_{g,t} = P_{g,\min} \cdot u_{g,h(t)} + \sum_k q_{g,k,t}$$

**Segment upper bounds**:
$$0 \le q_{g,k,t} \le Q_{g,k}$$

**Capacity limits**:
$$P_{g,\min} \cdot u_{g,h} \le p_{g,t} \le P_{g,\max} \cdot u_{g,h}$$

**Commitment transition** (for $h \ge 1$):
$$u_{g,h} - u_{g,h-1} = v_{g,h} - w_{g,h}, \quad v_{g,h} + w_{g,h} \le 1$$

For $h = 0$: $u_{g,0} - u_{g,-1}^0 = v_{g,0} - w_{g,0}$ where $u_{g,-1}^0$ is the initial status.

**Minimum up-time** (for each $h$):
$$\sum_{\tau=\max(0,h-L_g^{up}+1)}^{h} v_{g,\tau} \le u_{g,h}$$

**Minimum down-time** (for each $h$):
$$\sum_{\tau=\max(0,h-L_g^{dn}+1)}^{h} w_{g,\tau} \le 1 - u_{g,h}$$

**Ramp-up** (for $t \ge 1$):
$$p_{g,t} - p_{g,t-1} \le \text{RU}_g \cdot \Delta t + P_{g,\max} \cdot v_{g,h(t)}$$

**Ramp-down** (for $t \ge 1$):
$$p_{g,t-1} - p_{g,t} \le \text{RD}_g \cdot \Delta t + P_{g,\max} \cdot w_{g,h(t)}$$

### 8.4 Storage Constraints

**SOC dynamics** (for $t \ge 1$):
$$E_{s,t} = E_{s,t-1} + \eta p_{s,t}^{ch} \Delta t - \frac{1}{\eta} p_{s,t}^{dis} \Delta t$$

where $\eta = \sqrt{\eta_{rt}}$ (square root of round-trip efficiency).

For $t = 0$: $E_{s,0} = E_{s,-1}^0 + \eta p_{s,0}^{ch} \Delta t - \frac{1}{\eta} p_{s,0}^{dis} \Delta t$.

**Bounds**: $0.1 \cdot E_s^{\max} \le E_{s,t} \le E_s^{\max}$

**Simultaneous charge/discharge prevention**:
$$p_{s,t}^{ch} + p_{s,t}^{dis} \le \max(P_s^{ch,\max}, P_s^{dis,\max})$$

**Terminal SOC**: $E_{s,T} \ge E_{s,0}^0$

**Cycle limit** (when `cycle_limit > 0`):
$$\sum_t \left( \frac{p_{s,t}^{dis} \Delta t}{\eta} + \eta p_{s,t}^{ch} \Delta t \right) \le 2 \cdot n_c \cdot E_s^{\max}$$

### 8.5 Transmission Constraints

PTDF-based DC power flow:

$$\mathbf{F}^{\text{PTDF}} = \mathbf{B}_f \cdot \mathbf{B}_{\text{red}}^{-1}$$

where $\mathbf{B}_{\text{red}}$ is the reduced nodal susceptance matrix (slack bus removed),
computed via full-pivoting LU factorisation.

**Line flow definition** (equality for each $l, t$):
$$f_{l,t} = \sum_n \text{PTDF}_{l,n} \cdot P_{n,t}^{\text{net}}$$

where $P_{n,t}^{\text{net}}$ is the net injection at bus $n$ at time $t$ (generation minus load).

**Line flow limits** with elastic slacks:
$$-F_l^{\max} - s_{l,t}^- \le f_{l,t} \le F_l^{\max} + s_{l,t}^+, \quad s_{l,t}^{\pm} \ge 0$$

The slacks are penalised at rate $M_1$ (default $10^5$ $/MW-period$) to allow
infeasible solutions to be recovered rather than declaring the problem infeasible.

### 8.6 Power Balance

System-wide energy balance (equality for each $t$):

$$\sum_g p_{g,t} + \sum_w p_{w,t}^W + \sum_s p_{s,t}^{PV} + \sum_s p_{s,t}^{dis} + s_t^+ = \sum_d D_{d,t} + \sum_s p_{s,t}^{ch} + s_t^-$$

DC line injections appear on both sides of the balance according to their
`from`/`to` bus assignments.

### 8.7 Reserve Requirements

Spinning reserve (system total, for each $t$):
$$\sum_g r_{g,t}^{sp} \ge \sigma^{sp} \cdot \sum_d D_{d,t}$$

Regulation up:
$$\sum_g r_{g,t}^{up} \ge \sigma^{up} \cdot \sum_d D_{d,t}$$

Regulation down:
$$\sum_g r_{g,t}^{dn} \ge \sigma^{dn} \cdot \sum_d D_{d,t}$$

Reserve headroom constraints:
$$p_{g,t} + r_{g,t}^{sp} + r_{g,t}^{up} \le P_{g,\max} \cdot u_{g,h(t)}$$
$$p_{g,t} - r_{g,t}^{dn} \ge P_{g,\min} \cdot u_{g,h(t)}$$

### 8.8 Market Cutting Planes

When `enable_market_cuts = true`, the following LP-valid strengthening cuts are
added at formulation time.  These do not change the optimal solution but tighten
the LP relaxation and reduce the branch-and-bound tree.

| Family | Description |
|---|---|
| **Family 6** | Symmetry-breaking for generators with identical bid curves |
| **Family A** | Segment commitment coupling: $q_{g,k,t} \le Q_{g,k} \cdot u_{g,h(t)}$ |
| **Family G** | Extended startup clique: at most one startup in a min-up window |
| **Family 11** | Cyclic SOC bound for storage: ensures feasibility of the return constraint |

---

## 9. SCED Stage

When `config.solve_sced = true`, the SCED stage:

1. **Fixes** all binary commitment variables $u_{g,h}, v_{g,h}, w_{g,h}$ to
   their SCUC solution values by setting $\text{lb} = \text{ub} = x^*$.
2. **Builds** the same LP/MILP formulation but as a pure LP (all continuous).
3. **Solves** the LP with the configured solver.

SCED refines the continuous dispatch to account for any LP-relaxation
inaccuracies in the SCUC solve.  The commitment schedule is guaranteed to
match the SCUC result.

---

## 10. LMP Computation

When `config.solve_lmp = true`, the LMP stage computes nodal marginal prices
by solving a delta-neighbourhood LP:

1. **Build** the SCED LP with the SCUC commitment fixed.
2. **Apply delta-neighbourhood** bounds: for each generator $g$ at period $t$,
   allow dispatch to vary by $\pm \delta \cdot P_{g,\max}$ around the SCED solution
   ($\delta$ = `lmp_delta`, default 0.10).
3. **Solve** the LP, requesting constraint dual variables.
4. **Extract** power-balance equality constraint duals as the system energy LMP:
   $$\lambda_t = \mu_t^{\text{balance}} \quad [\$/\text{MWh}]$$
5. **Decompose** into energy and congestion components using PTDF duals:
   $$\text{LMP}_{n,t} = \lambda_t - \sum_l \text{PTDF}_{l,n} \cdot \mu_{l,t}^{\text{flow}}$$

**Dual source preference**: to ensure `constraint_duals` are populated, the LMP
stage tries solvers in order: `Gurobi` → `NativeBranchAndCut` → `NativeIPMLPAdapter`.
The HiGHS adapter is skipped because it does not return constraint duals.

**Dual layout** in `result.constraint_duals`:
```
[ ineq_dual_0, …, ineq_dual_{m_ineq-1},
  eq_dual_0, …, eq_dual_{m_eq-1} ]
```

Power-balance equality duals are at index `n_ineq + power_balance_eq_start + t`.
Line flow inequality duals are tracked per row index during formulation.

---

## 11. Solver Selection

`config.solver` accepts the following values (case-sensitive):

| Value | Adapter | Notes |
|---|---|---|
| `"Auto"` | Engine heuristic | MILP priority: StrictHiGHS → HiGHS → Gurobi → NativeBranchAndCut |
| `"StrictHiGHS"` | StrictHighsBranchAndCutAdapter | Embedded HiGHS state machine with MIPSolvers production contract; large roots use IPM with crossover by default |
| `"Gurobi"` | GurobiAdapter | Requires Gurobi licence; returns duals |
| `"HiGHS"` | HighsAdapter | Open-source; **no constraint duals** (LMP uses fallback) |
| `"SCIP"` | ScipAdapter | Open-source **MINLP only** (not registered for MILP dispatch) |
| `"NativeBranchAndCut"` | NativeBranchAndCutAdapter | Built-in B&C; returns duals |

If `allow_fallback = true` (default) and the preferred solver fails or is
unavailable, the engine automatically tries the next registered adapter.

To query available solvers at runtime:

```cpp
mipsolvers::engine::SolverEngine eng;
eng.register_default_adapters();
for (const auto& s : eng.list_solvers(mipsolvers::engine::ProblemClass::MILP))
    std::cout << s << "\n";
```

---

## 12. CLI Tool

The `scuc_solve` executable is built when `MIPSOLVERS_BUILD_SCUC=ON` (default).

```
Usage:
  scuc_solve <input.json> [output.json]
             [--solver <Auto|StrictHiGHS|Gurobi|HiGHS|SCIP|NativeBranchAndCut>]
             [--no-sced]
             [--no-lmp]
             [--indent <n>]

Arguments:
  input.json            Path to JSON input file (required)
  output.json           Path for JSON output (optional; stdout if omitted)
  --solver <name>       Override config.solver
  --no-sced             Skip SCED LP stage
  --no-lmp              Skip LMP computation
  --indent <n>          JSON indentation (default: 2; 0 = compact)
  -h, --help            Print usage and list available solvers

Examples:
  scuc_solve day_ahead.json results.json --solver HiGHS
  scuc_solve day_ahead.json --solver Auto --no-lmp --indent 0
  scuc_solve day_ahead.json | python3 postprocess.py
```

Exit code 0 = SCUC converged; non-zero = parse error or solver failure.

---

## 13. Build Integration

The SCUC module is **not** part of `libmipsolvers.a`.  It must be compiled
separately (because it depends on nlohmann/json and the solver engine, but
is not a core solver component).

### CMake — executable

```cmake
# Controlled by option MIPSOLVERS_BUILD_SCUC (default ON)
add_executable(scuc_solve
  src/scuc/main.cpp
  src/scuc/scuc.cpp
)
target_include_directories(scuc_solve PRIVATE include)
target_link_libraries(scuc_solve PRIVATE mipsolvers)
```

### CMake — library target (custom)

To embed SCUC in your own executable or library:

```cmake
add_library(my_scuc STATIC
  <path/to/mipsolvers>/src/scuc/scuc.cpp
)
target_include_directories(my_scuc PUBLIC
  <path/to/mipsolvers>/include
)
target_link_libraries(my_scuc PUBLIC mipsolvers)

# Use:
target_link_libraries(my_app PRIVATE my_scuc)
```

### Minimal JSON input for smoke-testing

```json
{
  "config":      { "solver": "Auto", "num_periods": 3 },
  "num_buses":   1,
  "generators":  [{"name":"G","bus":0,"pmin":0,"pmax":100,
                   "bid_segments":[{"price":30,"quantity":100}]}],
  "branches":    [],
  "loads":       [{"bus":0,"p_mw":80}],
  "wind":        [],
  "solar":       [],
  "storage":     [],
  "dc_lines":    [],
  "profiles":    {"load":[[1.0,1.0,1.0]],"wind":[],"solar":[]},
  "initial_status": {"commitment":[1.0],"dispatch":[80.0],"storage_soc":[]}
}
```

---

## 14. Test Case Generators

The `case_builder` module provides three pre-built `SCUCInput` objects for
unit testing, integration testing, and benchmarking.

```cpp
#include "mipsolvers/scuc/case_builder.hpp"

namespace mipsolvers::scuc {

/// Tiny 3-bus / 2-generator case for fast smoke tests.
SCUCInput build_3bus_case(int T = 3, double dt = 1.0);

/// Standard 6-bus IEEE case with optional wind and storage.
SCUCInput build_6bus_case(int T = 24, double dt = 1.0,
                          bool with_wind    = false,
                          bool with_storage = false);

/// IEEE 39-bus New England system — 10 generators, 51 branches,
/// 17 load buses (4 599 MW base), optional wind / solar.
SCUCInput build_ieee39_case(int T = 24, double dt = 1.0,
                            bool with_wind  = false,
                            bool with_solar = false);

/// Serialize an SCUCInput to JSON (indent = -1 for compact).
std::string scuc_input_to_json(const SCUCInput& inp, int indent = 2);

} // namespace mipsolvers::scuc
```

### Case summary

| Function | Buses | Generators | Branches | Optional resources |
|---|---|---|---|---|
| `build_3bus_case` | 3 | 2 (coal + gas) | 2 | — |
| `build_6bus_case` | 6 | 3 | 7 | wind (bus 3, 100 MW); storage (bus 5, 240 MWh) |
| `build_ieee39_case` | 39 | 10 | 51 | wind (3 sites, 1 050 MW); solar (2 sites, 450 MW) |

### IEEE 39-bus system details

**Generators** (0-indexed buses):

| Generator | Bus | P_min (MW) | P_max (MW) | Startup cost ($) |
|---|---|---|---|---|
| G1 | 38 | 250 | 1 100 | 15 000 |
| G2 | 30 | 100 | 650 | 8 000 |
| G3 | 31 | 150 | 725 | 9 000 |
| G4 | 32 | 150 | 650 | 9 000 |
| G5 | 33 | 100 | 508 | 7 500 |
| G6 | 34 | 50 | 687 | 8 500 |
| G7 | 35 | 100 | 580 | 8 000 |
| G8 | 36 | 50 | 564 | 6 000 |
| G9 | 37 | 50 | 865 | 9 000 |
| G10 | 29 | 300 | 1 100 | 16 000 |

**Branch count**: 51 total = 41 transmission lines + 2 G10 tie lines +
1 G9 step-up transformer + 7 G2–G8 step-up transformers.

**Default SCUC config**: `spinning_reserve_req = 5%`, `regulation_up_req = 3%`,
`regulation_down_req = 2%`, `pfr_reserve_req_mw = 300`, `voll = 10 000`,
`vocc = 300`, `enable_market_cuts = true`.

**Initial commitment**: G1, G2, G5, G10 online at {500, 300, 200, 600} MW.

---

## 15. Solver Benchmarks

Environment: macOS ARM64 (Apple M4), Release build, MIP gap 1%, HiGHS 4.x.

| Case | T | Resources | HiGHS | Gurobi | NativeBranchAndCut | Objective ($) | Cuts |
|---|---|---|---|---|---|---|---|
| 3-bus | 6 | — | 22 ms | 4 ms | 48 ms | 746,316 | 32 |
| 6-bus | 8 | wind+storage | 22 ms | 6 ms | 70 ms | 76,478 | 86 |
| IEEE 39-bus | 4 | wind | 21 ms | 5 ms | 16 ms | 203,130 | 122 |
| IEEE 39-bus | 24 | wind+solar | 91 ms | 77 ms | 96 ms | 891,466 | 898 |
| **IEEE 39-bus** | **24** | **wind+solar+storage** | **96 ms** | **84 ms** | **147 ms** | **891,466** | **900** |

> **Notes:**
> - SCIP supports MINLP only and is not registered for MILP dispatch in this
>   framework.
> - All three solvers agree on objective values (differences < 1 $ within MIP
>   tolerance).
> - `"Auto"` mode selects StrictHiGHS first for MILP, then falls back to
>   HiGHS → Gurobi → NativeBranchAndCut.
> - Zero load-shed for all IEEE 39-bus cases (VOLL penalty = $0).
> - The wind+solar case objective (891,466 $) is lower than wind-only (976,359 $):
>   solar generation displaces expensive peak-period thermal capacity.
> - The wind+solar+storage case adds two batteries (bus 3: 200 MW/800 MWh;
>   bus 19: 150 MW/600 MWh); same objective, two additional SOC cuts.

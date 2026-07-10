Below is a **technical implementation Markdown specification** you can give directly to Claude or another engineering agent to reproduce the hosting-capacity assessment workflow based on **DL/T 2041—2025**. It assumes you already have mature modules for **power flow**, **short-circuit calculation**, **harmonic power flow**, and related grid-analysis functions.

---

# Technical Implementation Specification  
# Distributed Resource Hosting Capacity Assessment Based on DL/T 2041—2025

## 1. Purpose and Scope

This document defines a software implementation framework for evaluating the **hosting capacity of distributed resources connected to a power system**, following the methodology of **DL/T 2041—2025**.

The implementation covers:

1. **System-level hosting capacity calculation**
2. **Equipment-level hosting capacity calculation**
3. **County-level result reconciliation between system-level and equipment-level capacity**
4. **Accessible capacity evaluation**
5. **Green / Yellow / Red grading**
6. **Optional engineering verification using existing power-flow, short-circuit, voltage-deviation, and harmonic-analysis modules**

The system is intended for use by utilities, grid companies, planning departments, and distributed energy resource access-management platforms.

---

## 2. Core Concepts

### 2.1 Distributed Resource

A distributed resource refers to a generation resource built near users, mainly intended for local consumption, usually connected to voltage levels of **35 kV and below**.

In this implementation, the following shall also be treated as distributed resources:

- Large industrial and commercial distributed PV connected at **110 kV / 66 kV**, with total capacity generally not exceeding **50 MW**
- Hydropower units of **50 MW and below**
- Other inverter-based, synchronous-generator-based, or induction-generator-based distributed resources

---

### 2.2 Hosting Capacity

Hosting capacity refers to the maximum installed capacity of distributed resources that can be connected while satisfying:

- Power-system security and stability requirements
- Equipment loading limits
- Reverse power-flow constraints
- Power-quality requirements
- Renewable-energy utilization targets
- Flexibility-resource constraints

Hosting capacity is divided into:

| Type | Description |
|---|---|
| **System-level hosting capacity** | Maximum distributed-resource capacity that can be connected within a provincial administrative region, independent dispatch-control area, or county-level area |
| **Equipment-level hosting capacity** | Maximum distributed-resource capacity that can be connected within the supply area of a transformer at 220/330 kV and below |

---

### 2.3 Accessible Capacity

Accessible capacity refers to the additional distributed-resource capacity that can be newly connected after deducting:

1. Already grid-connected distributed-resource capacity
2. Already registered but not yet grid-connected distributed-resource capacity

Two accessible-capacity concepts are used:

| Capacity Type | Meaning |
|---|---|
| **Accessible grid-connection capacity** | Remaining capacity after deducting already grid-connected distributed resources |
| **Accessible registration capacity** | Remaining capacity after deducting both already grid-connected and already registered-but-not-connected distributed resources |

---

### 2.4 Renewable-Energy Utilization Rate

The renewable-energy utilization rate is defined as:

$$
\eta_{RE} = \frac{E_{RE,actual}}{E_{RE,available}}
$$

Where:

- $$\eta_{RE}$$ = renewable-energy utilization rate
- $$E_{RE,actual}$$ = actual renewable-energy generation
- $$E_{RE,available}$$ = available renewable-energy generation

All curtailed energy caused by any reason should be included when calculating the utilization rate.

---

### 2.5 Reverse Load Rate

The reverse load rate of a transformer is defined as:

$$
R_{reverse} = \frac{P_{reverse}}{S_{rated}}
$$

Where:

- $$R_{reverse}$$ = reverse load rate
- $$P_{reverse}$$ = reverse active power flowing from the lower-voltage side to the higher-voltage side
- $$S_{rated}$$ = rated transformer capacity

Power flowing from the lower-voltage side to the higher-voltage side is considered **reverse power flow**.

---

### 2.6 Maximum Output Coefficient of Distributed Resources

The maximum output coefficient of distributed resources is defined as:

$$
\tau_{max} = \frac{\sum P_{DR,max}}{\sum S_{DR,rated}}
$$

Where:

- $$\tau_{max}$$ = maximum output coefficient
- $$P_{DR,max}$$ = maximum simultaneous output of distributed resources within the evaluated area
- $$S_{DR,rated}$$ = rated installed capacity of distributed resources within the evaluated area

---

## 3. Overall Assessment Workflow

The full implementation shall follow this workflow:

```text
1. Load source data
2. Build system-level simulation model
3. Calculate provincial / dispatch-area system-level hosting capacity
4. Decompose system-level hosting capacity to county-level areas
5. Calculate equipment-level hosting capacity for transformers
6. Aggregate equipment-level capacity to county-level areas
7. Reconcile county-level system and equipment results
8. Calculate accessible grid-connection capacity
9. Calculate accessible registration capacity
10. Assign Green / Yellow / Red assessment grades
11. Perform optional power-flow, short-circuit, voltage-deviation, and harmonic verification
12. Generate reports and data outputs
```

---

## 4. Data Model

### 4.1 Administrative Area Model

```typescript
interface AdministrativeArea {
  id: string;
  name: string;
  type: "province" | "dispatch_area" | "city" | "county";
  parentId?: string;

  annualPeakLoadMW?: number;
  annualElectricityConsumptionMWh?: number;
  userCount?: number;
  gdp?: number;

  existingDistributedResourceMW?: number;
  newlyAddedDistributedResourceMW?: number;
  registeredDistributedResourceMW?: number;
  technicalDevelopablePotentialMW?: number;

  systemHostingCapacityMinMW?: number;
  systemHostingCapacityMaxMW?: number;

  equipmentHostingCapacityMinMW?: number;
  equipmentHostingCapacityMaxMW?: number;

  reconciledHostingCapacityMinMW?: number;
  reconciledHostingCapacityMaxMW?: number;

  accessibleGridConnectionCapacityMinMW?: number;
  accessibleGridConnectionCapacityMaxMW?: number;

  accessibleRegistrationCapacityMinMW?: number;
  accessibleRegistrationCapacityMaxMW?: number;

  grade?: "green" | "yellow" | "red";
}
```

---

### 4.2 Transformer Model

```typescript
interface Transformer {
  id: string;
  name: string;

  voltageLevel: "330kV" | "220kV" | "110kV" | "66kV" | "35kV" | "10kV";
  parentTransformerId?: string;
  countyId: string;

  ratedCapacityMVA: number;
  powerFactor: number;

  operationMode: "single" | "parallel" | "split";
  transformerCountInStation?: number;
  nMinusOneConstrained?: boolean;

  maxReverseLoadRate: number;

  typicalLoadMW: number;
  nonDistributedGenerationOutputMW: number;

  existingStorageChargingPowerMW: number;
  expectedNewStorageChargingPowerMinMW: number;
  expectedNewStorageChargingPowerMaxMW: number;

  distributedResourceMaxOutputCoefficient: number;

  existingDistributedResourceMW: number;
  registeredDistributedResourceMW: number;

  equipmentHostingCapacityMinMW?: number;
  equipmentHostingCapacityMaxMW?: number;

  accessibleGridConnectionCapacityMinMW?: number;
  accessibleGridConnectionCapacityMaxMW?: number;

  accessibleRegistrationCapacityMinMW?: number;
  accessibleRegistrationCapacityMaxMW?: number;

  grade?: "green" | "yellow" | "red";
}
```

---

### 4.3 System Simulation Input Model

```typescript
interface SystemSimulationInput {
  areaId: string;
  assessmentYear: number;

  renewableUtilizationTarget: number;

  hourlyLoadCurveMW: number[];

  coalGeneration: GenerationFleet;
  gasGeneration: GenerationFleet;
  hydroGeneration: GenerationFleet;
  nuclearGeneration: GenerationFleet;
  windGeneration: RenewableFleet;
  photovoltaicGeneration: RenewableFleet;
  biomassGeneration: GenerationFleet;

  pumpedStorage: StorageFleet;
  newEnergyStorage: StorageFleet;

  crossProvinceTransmission: TransmissionSchedule[];

  reserveRequirementMW?: number[];
  securityConstraints?: SecurityConstraint[];

  existingDistributedResourceMW: number;
}
```

---

### 4.4 Generation Fleet Model

```typescript
interface GenerationFleet {
  installedCapacityMW: number;
  minOutputMW?: number;
  maxOutputMW?: number;
  hourlyAvailableOutputMW?: number[];
  hourlyScheduledOutputMW?: number[];
  rampRateMWPerHour?: number;
  forcedOutageRate?: number;
  maintenanceSchedule?: boolean[];
}
```

---

### 4.5 Renewable Fleet Model

```typescript
interface RenewableFleet {
  installedCapacityMW: number;
  hourlyAvailableOutputMW: number[];
  hourlyActualOutputMW?: number[];
}
```

---

### 4.6 Storage Fleet Model

```typescript
interface StorageFleet {
  powerCapacityMW: number;
  energyCapacityMWh: number;
  initialStateOfChargeMWh?: number;
  minStateOfChargeMWh?: number;
  maxStateOfChargeMWh?: number;
  chargingEfficiency?: number;
  dischargingEfficiency?: number;
  hourlyChargingPowerMW?: number[];
  hourlyDischargingPowerMW?: number[];
}
```

---

### 4.7 Transmission Schedule Model

```typescript
interface TransmissionSchedule {
  id: string;
  name: string;
  direction: "import" | "export";
  hourlyPowerMW: number[];
}
```

---

## 5. System-Level Hosting Capacity Calculation

## 5.1 Calculation Objective

The system-level calculation determines the maximum distributed-resource installed capacity that can be accepted by a provincial administrative area or independent dispatch-control area while satisfying:

- Renewable-energy utilization target
- Power balance
- Reserve requirements
- Generation operation constraints
- Storage operation constraints
- Cross-region transmission constraints
- Security and stability constraints

The result is a hosting-capacity interval:

$$
S_S = [S_{S,min}, S_{S,max}]
$$

Where:

- $$S_S$$ = system-level hosting capacity
- $$S_{S,min}$$ = lower bound of system-level hosting capacity
- $$S_{S,max}$$ = upper bound of system-level hosting capacity

---

## 5.2 Recommended Calculation Period

System-level calculation should usually be performed:

- Annually
- Or according to the power-system planning cycle
- Or whenever significant boundary conditions change

Significant boundary changes include:

- Large renewable-energy capacity additions
- Major load forecast changes
- New interconnection or transmission corridor commissioning
- New pumped-storage or battery-storage projects
- Major thermal-unit flexibility retrofits
- Renewable-utilization target adjustment
- Market-rule changes

---

## 5.3 Production Simulation Requirement

The implementation should use a chronological production-simulation model.

For hourly simulation:

$$
t = 1, 2, 3, \ldots, 8760
$$

At every time step, the power balance should satisfy:

$$
P_{coal,t}
+ P_{gas,t}
+ P_{hydro,t}
+ P_{nuclear,t}
+ P_{wind,t}
+ P_{pv,t}
+ P_{biomass,t}
+ P_{storage,dis,t}
+ P_{import,t}
=
P_{load,t}
+ P_{storage,ch,t}
+ P_{export,t}
+ P_{curtail,t}
$$

The implementation may simplify this equation depending on available production-simulation capability, but it must still calculate renewable-energy utilization.

---

## 5.4 Renewable-Energy Utilization Constraint

For each simulated scenario, calculate:

$$
\eta_{RE} =
\frac{
\sum_t E_{RE,actual,t}
}
{
\sum_t E_{RE,available,t}
}
$$

The scenario is acceptable if:

$$
\eta_{RE} \geq \eta_{target}
$$

Where:

- $$\eta_{target}$$ = renewable-energy utilization target issued by the relevant authority

---

## 5.5 Iterative Capacity Search

The basic algorithm is:

```text
1. Use existing grid-connected distributed-resource capacity as the base scenario.
2. Run chronological production simulation.
3. Calculate renewable-energy utilization rate.
4. Increase distributed-resource installed capacity.
5. Rerun production simulation.
6. Repeat until renewable-energy utilization equals or just meets the target.
7. The corresponding installed distributed-resource capacity is the system-level hosting capacity.
```

Use binary search or monotonic iterative search.

---

### 5.5.1 Binary Search Pseudocode

```python
def calculate_system_hosting_capacity(
    simulation_input,
    renewable_utilization_target,
    storage_capacity_mw,
    lower_bound_mw,
    upper_bound_mw,
    tolerance_mw=1.0,
    utilization_tolerance=0.0001
):
    low = lower_bound_mw
    high = upper_bound_mw
    best_feasible_capacity = low

    while high - low > tolerance_mw:
        mid = (low + high) / 2

        scenario = build_scenario(
            simulation_input=simulation_input,
            distributed_resource_capacity_mw=mid,
            storage_capacity_mw=storage_capacity_mw
        )

        result = run_chronological_production_simulation(scenario)

        renewable_utilization = calculate_renewable_utilization(result)

        if renewable_utilization + utilization_tolerance >= renewable_utilization_target:
            best_feasible_capacity = mid
            low = mid
        else:
            high = mid

    return best_feasible_capacity
```

---

## 5.6 System-Level Capacity Interval

The system-level hosting capacity shall be calculated as an interval, not only a single value.

The main variables are:

1. Renewable-energy utilization target
2. New-energy storage capacity

---

### 5.6.1 Annual Assessment

For annual assessment:

- Use the annual renewable-energy utilization target as fixed.
- Use new-energy storage capacity as the variable.

Let:

$$
S_{ESS} \in [S_{ESS,min}, S_{ESS,max}]
$$

Then:

$$
S_{S,min} = f(\eta_{target}, S_{ESS,min})
$$

$$
S_{S,max} = f(\eta_{target}, S_{ESS,max})
$$

Where:

- $$f$$ = chronological production simulation and iterative capacity-search function

Implementation:

```python
def calculate_annual_system_capacity_interval(input_data):
    target = input_data.renewableUtilizationTarget

    storage_min = input_data.storageCapacityMinMW
    storage_max = input_data.storageCapacityMaxMW

    capacity_min = calculate_system_hosting_capacity(
        simulation_input=input_data,
        renewable_utilization_target=target,
        storage_capacity_mw=storage_min,
        lower_bound_mw=input_data.existingDistributedResourceMW,
        upper_bound_mw=input_data.searchUpperBoundMW
    )

    capacity_max = calculate_system_hosting_capacity(
        simulation_input=input_data,
        renewable_utilization_target=target,
        storage_capacity_mw=storage_max,
        lower_bound_mw=input_data.existingDistributedResourceMW,
        upper_bound_mw=input_data.searchUpperBoundMW
    )

    return {
        "systemHostingCapacityMinMW": capacity_min,
        "systemHostingCapacityMaxMW": capacity_max
    }
```

---

### 5.6.2 Planning-Cycle Assessment

For planning-cycle assessment:

- Renewable-energy utilization target is also treated as a variable.
- New-energy storage capacity is treated as a variable.

Usually:

$$
\eta_{target} \in [\eta_{low}, \eta_{high}]
$$

$$
S_{ESS} \in [S_{ESS,min}, S_{ESS,max}]
$$

The lower capacity bound should be calculated under:

- Higher renewable-utilization requirement
- Lower storage capacity

The upper capacity bound should be calculated under:

- Lower renewable-utilization requirement
- Higher storage capacity

Therefore:

$$
S_{S,min} = f(\eta_{high}, S_{ESS,min})
$$

$$
S_{S,max} = f(\eta_{low}, S_{ESS,max})
$$

Implementation:

```python
def calculate_planning_cycle_system_capacity_interval(input_data):
    eta_low = input_data.renewableUtilizationTargetLow
    eta_high = input_data.renewableUtilizationTargetHigh

    storage_min = input_data.storageCapacityMinMW
    storage_max = input_data.storageCapacityMaxMW

    capacity_min = calculate_system_hosting_capacity(
        simulation_input=input_data,
        renewable_utilization_target=eta_high,
        storage_capacity_mw=storage_min,
        lower_bound_mw=input_data.existingDistributedResourceMW,
        upper_bound_mw=input_data.searchUpperBoundMW
    )

    capacity_max = calculate_system_hosting_capacity(
        simulation_input=input_data,
        renewable_utilization_target=eta_low,
        storage_capacity_mw=storage_max,
        lower_bound_mw=input_data.existingDistributedResourceMW,
        upper_bound_mw=input_data.searchUpperBoundMW
    )

    return {
        "systemHostingCapacityMinMW": capacity_min,
        "systemHostingCapacityMaxMW": capacity_max
    }
```

---

## 6. Decomposition of System-Level Capacity to Counties

## 6.1 Objective

After obtaining the system-level capacity interval for a province or dispatch-control area, decompose the result to county-level administrative areas.

For each county $$i$$:

$$
S_{S,min,i} = w_i \times S_{S,min,total}
$$

$$
S_{S,max,i} = w_i \times S_{S,max,total}
$$

Where:

- $$w_i$$ = allocation weight of county $$i$$
- $$S_{S,min,total}$$ = total system-level lower-bound capacity
- $$S_{S,max,total}$$ = total system-level upper-bound capacity

---

## 6.2 County Allocation Weight

The allocation weight may be calculated using a multi-factor weighted method.

For annual assessment, recommended factors include:

- Electricity load
- Electricity consumption
- Number of users
- Existing distributed-resource capacity
- Newly added distributed-resource capacity
- Registered distributed-resource capacity
- GDP

For planning-cycle assessment, additional factors may include:

- Technical developable distributed-resource potential
- Pumped-storage capacity
- Planned storage capacity
- Grid-upgrade potential

---

## 6.3 Annual Weight Formula

For county $$i$$:

$$
w_i =
a_1 r_{load,i}
+ a_2 r_{energy,i}
+ a_3 r_{user,i}
+ a_4 r_{existingDR,i}
+ a_5 r_{newDR,i}
+ a_6 r_{registeredDR,i}
+ a_7 r_{gdp,i}
$$

Where:

$$
a_1 + a_2 + a_3 + a_4 + a_5 + a_6 + a_7 = 1
$$

Each ratio is calculated as:

$$
r_{x,i} = \frac{x_i}{\sum_j x_j}
$$

---

## 6.4 Suggested Default Weights

The standard does not mandate exact weighting coefficients. The implementation should make them configurable.

Recommended default:

| Factor | Symbol | Default Weight |
|---|---:|---:|
| Peak load | $$a_1$$ | 0.25 |
| Electricity consumption | $$a_2$$ | 0.20 |
| User count | $$a_3$$ | 0.10 |
| Existing distributed resources | $$a_4$$ | 0.15 |
| Newly added distributed resources | $$a_5$$ | 0.10 |
| Registered distributed resources | $$a_6$$ | 0.10 |
| GDP | $$a_7$$ | 0.10 |

The implementation shall allow users to override these weights.

---

## 6.5 Decomposition Pseudocode

```python
def calculate_county_weights(counties, weights):
    totals = {
        "load": sum(c.annualPeakLoadMW or 0 for c in counties),
        "energy": sum(c.annualElectricityConsumptionMWh or 0 for c in counties),
        "users": sum(c.userCount or 0 for c in counties),
        "existingDR": sum(c.existingDistributedResourceMW or 0 for c in counties),
        "newDR": sum(c.newlyAddedDistributedResourceMW or 0 for c in counties),
        "registeredDR": sum(c.registeredDistributedResourceMW or 0 for c in counties),
        "gdp": sum(c.gdp or 0 for c in counties)
    }

    result = {}

    for c in counties:
        r_load = safe_ratio(c.annualPeakLoadMW, totals["load"])
        r_energy = safe_ratio(c.annualElectricityConsumptionMWh, totals["energy"])
        r_users = safe_ratio(c.userCount, totals["users"])
        r_existing = safe_ratio(c.existingDistributedResourceMW, totals["existingDR"])
        r_new = safe_ratio(c.newlyAddedDistributedResourceMW, totals["newDR"])
        r_registered = safe_ratio(c.registeredDistributedResourceMW, totals["registeredDR"])
        r_gdp = safe_ratio(c.gdp, totals["gdp"])

        w = (
            weights["load"] * r_load
            + weights["energy"] * r_energy
            + weights["users"] * r_users
            + weights["existingDR"] * r_existing
            + weights["newDR"] * r_new
            + weights["registeredDR"] * r_registered
            + weights["gdp"] * r_gdp
        )

        result[c.id] = w

    normalized = normalize_weights(result)
    return normalized
```

---

```python
def decompose_system_capacity_to_counties(
    counties,
    total_system_capacity_min_mw,
    total_system_capacity_max_mw,
    weights
):
    county_weights = calculate_county_weights(counties, weights)

    for c in counties:
        w = county_weights[c.id]

        c.systemHostingCapacityMinMW = w * total_system_capacity_min_mw
        c.systemHostingCapacityMaxMW = w * total_system_capacity_max_mw

    return counties
```

---

## 7. Equipment-Level Hosting Capacity Calculation

## 7.1 Calculation Objective

The equipment-level hosting capacity calculation determines the maximum distributed-resource capacity that can be connected within the supply area of each transformer while keeping the reverse load rate within the allowed limit.

The target equipment includes:

- 330 kV transformers
- 220 kV transformers
- 110 kV transformers
- 66 kV transformers
- 35 kV transformers
- 10 kV transformers

---

## 7.2 Calculation Period

Equipment-level hosting capacity should usually be calculated quarterly.

The assessment period may be shortened when:

- Distributed PV capacity grows rapidly
- Network topology changes
- Load changes significantly
- Storage or flexibility resources are added
- Yellow or red warning zones exist
- New registration applications are concentrated

---

## 7.3 Typical Time Selection

The typical time should be selected from:

- Spring or autumn
- Working days
- Noon period
- Maximum distributed-resource output condition

The implementation should support both:

1. Manual typical-time selection
2. Automatic typical-time selection from historical data

---

### 7.3.1 Automatic Typical-Time Selection

Recommended logic:

```text
1. Filter historical timestamps to spring or autumn.
2. Filter to working days.
3. Filter to noon period, for example 10:00–14:00.
4. Calculate distributed-resource total output for each candidate timestamp.
5. Select the timestamp with maximum distributed-resource output.
6. Use the corresponding transformer load and local generation data as inputs.
```

Pseudocode:

```python
def select_typical_timestamp(time_series):
    candidates = []

    for record in time_series:
        if not is_spring_or_autumn(record.timestamp):
            continue

        if not is_working_day(record.timestamp):
            continue

        if not is_noon_period(record.timestamp):
            continue

        candidates.append(record)

    if not candidates:
        raise ValueError("No valid typical timestamp found.")

    selected = max(
        candidates,
        key=lambda r: r.distributedResourceOutputMW
    )

    return selected.timestamp
```

---

## 7.4 Equipment-Level Formula

For each transformer, calculate:

$$
S_d =
\frac{
P - P_G + \beta \times S \times cos\theta + P_{ESS} + \Delta P_{ESS}
}
{\tau_{max}}
$$

Where:

| Symbol | Description |
|---|---|
| $$S_d$$ | Equipment-level hosting capacity of one transformer |
| $$P$$ | Load within the transformer supply area at the typical time |
| $$P_G$$ | Output of non-distributed generation within the transformer supply area at the typical time |
| $$S$$ | Rated capacity of the transformer |
| $$cos\theta$$ | Transformer power factor |
| $$\beta$$ | Maximum reverse load rate |
| $$\tau_{max}$$ | Maximum output coefficient of distributed resources |
| $$P_{ESS}$$ | Charging power of existing flexibility resources at the typical time |
| $$\Delta P_{ESS}$$ | Expected additional flexibility-resource charging power interval |

---

## 7.5 Equipment-Level Capacity Interval

Because expected new flexibility resources are represented as an interval:

$$
\Delta P_{ESS} \in [\Delta P_{ESS,min}, \Delta P_{ESS,max}]
$$

The hosting capacity interval is:

$$
S_{d,min} =
\frac{
P - P_G + \beta S cos\theta + P_{ESS} + \Delta P_{ESS,min}
}
{\tau_{max}}
$$

$$
S_{d,max} =
\frac{
P - P_G + \beta S cos\theta + P_{ESS} + \Delta P_{ESS,max}
}
{\tau_{max}}
$$

Thus:

$$
S_d = [S_{d,min}, S_{d,max}]
$$

---

## 7.6 Reverse Load Rate Parameter

The maximum reverse load rate $$\beta$$ should be selected as follows.

### 7.6.1 220/330 kV Transformers

| Condition | Recommended $$\beta$$ |
|---|---:|
| Only one transformer in operation | 0.80 |
| Two or more transformers in parallel operation | Determined by N-1 reverse-overload constraint |
| Split operation or distributed-resource tripping device installed | May be increased if justified |

---

### 7.6.2 110/66 kV and Below Transformers

| Condition | Recommended $$\beta$$ |
|---|---:|
| Split operation | 0.80 |
| Parallel operation | Determined by N-1 reverse-overload constraint |

---

## 7.7 N-1-Based Reverse Load Rate Calculation

If multiple transformers are operated in parallel, calculate the maximum allowable reverse load rate under N-1.

For a station with $$n$$ transformers, each transformer rated capacity $$S_i$$, and the evaluated transformer or station aggregate reverse power $$P_{reverse}$$:

1. Remove one transformer.
2. Redistribute reverse power to remaining transformers.
3. Check that no remaining transformer exceeds its allowed loading.
4. Determine maximum reverse power that satisfies all N-1 cases.

Simplified station-level formula for identical parallel transformers:

$$
\beta_{N-1} =
\frac{
(n - 1) \times S_{rated} \times loadingLimit
}
{
n \times S_{rated}
}
$$

So:

$$
\beta_{N-1} =
\frac{n - 1}{n} \times loadingLimit
$$

For example, if there are two identical transformers and the reverse loading limit under N-1 is 100%:

$$
\beta_{N-1} = \frac{2 - 1}{2} \times 1.0 = 0.5
$$

The implementation should support both:

- Manual $$\beta$$ input
- Automatic N-1 calculation

---

## 7.8 Equipment-Level Calculation Pseudocode

```python
def calculate_transformer_hosting_capacity(transformer):
    P = transformer.typicalLoadMW
    PG = transformer.nonDistributedGenerationOutputMW
    S = transformer.ratedCapacityMVA
    pf = transformer.powerFactor
    beta = transformer.maxReverseLoadRate
    tau = transformer.distributedResourceMaxOutputCoefficient
    P_ESS = transformer.existingStorageChargingPowerMW
    dP_ESS_min = transformer.expectedNewStorageChargingPowerMinMW
    dP_ESS_max = transformer.expectedNewStorageChargingPowerMaxMW

    if tau <= 0:
        raise ValueError("distributedResourceMaxOutputCoefficient must be greater than zero.")

    numerator_min = P - PG + beta * S * pf + P_ESS + dP_ESS_min
    numerator_max = P - PG + beta * S * pf + P_ESS + dP_ESS_max

    capacity_min = numerator_min / tau
    capacity_max = numerator_max / tau

    capacity_min = max(capacity_min, 0)
    capacity_max = max(capacity_max, 0)

    transformer.equipmentHostingCapacityMinMW = capacity_min
    transformer.equipmentHostingCapacityMaxMW = capacity_max

    return transformer
```

---

## 8. County-Level Equipment Capacity Aggregation

## 8.1 Aggregation Rule

For each county, the equipment-level hosting capacity should be calculated as the sum of the hosting-capacity intervals of all 220/330 kV transformers serving that county.

For county $$i$$:

$$
S_{D,min,i} = \sum_{k \in T_i} S_{d,min,k}
$$

$$
S_{D,max,i} = \sum_{k \in T_i} S_{d,max,k}
$$

Where:

- $$T_i$$ = set of 220/330 kV transformers serving county $$i$$
- $$S_D$$ = county-level equipment hosting capacity

---

## 8.2 Transformers Serving Multiple Counties

If a transformer serves multiple counties, its capacity should be allocated by a defined allocation ratio.

Possible allocation bases:

- Load ratio
- Electricity-consumption ratio
- Feeder ownership ratio
- Actual power-flow contribution
- Manually assigned supply responsibility ratio

For transformer $$k$$ serving county $$i$$ with allocation ratio $$\lambda_{k,i}$$:

$$
S_{D,min,i} += \lambda_{k,i} S_{d,min,k}
$$

$$
S_{D,max,i} += \lambda_{k,i} S_{d,max,k}
$$

Where:

$$
\sum_i \lambda_{k,i} = 1
$$

---

## 8.3 Fallback Rule

If a county has no 220/330 kV transformer:

Use one of the following methods:

1. Use the corresponding share of the external 220/330 kV transformer that supplies this county.
2. Use the sum of 110/66 kV transformer hosting capacities within the county.

The method should be configurable.

---

## 8.4 Aggregation Pseudocode

```python
def aggregate_equipment_capacity_to_counties(counties, transformers, supply_ratios):
    for county in counties:
        county.equipmentHostingCapacityMinMW = 0
        county.equipmentHostingCapacityMaxMW = 0

    for transformer in transformers:
        if transformer.voltageLevel not in ["330kV", "220kV"]:
            continue

        county_allocations = supply_ratios.get(transformer.id)

        if county_allocations:
            for county_id, ratio in county_allocations.items():
                county = find_county(counties, county_id)
                county.equipmentHostingCapacityMinMW += (
                    transformer.equipmentHostingCapacityMinMW * ratio
                )
                county.equipmentHostingCapacityMaxMW += (
                    transformer.equipmentHostingCapacityMaxMW * ratio
                )
        else:
            county = find_county(counties, transformer.countyId)
            county.equipmentHostingCapacityMinMW += transformer.equipmentHostingCapacityMinMW
            county.equipmentHostingCapacityMaxMW += transformer.equipmentHostingCapacityMaxMW

    return counties
```

---

## 9. Reconciliation Between System-Level and Equipment-Level Results

## 9.1 Principle

County-level reconciliation should follow the principle:

```text
Local capacity shall obey overall system capacity.
```

This means that the final county-level hosting capacity shall take the stricter result between:

- County-level system hosting capacity
- County-level equipment hosting capacity

---

## 9.2 Reconciliation Formula

For each county:

$$
S_i =
[
min(S_{S,min,i}, S_{D,min,i}),
min(S_{S,max,i}, S_{D,max,i})
]
$$

Where:

- $$S_i$$ = reconciled hosting-capacity interval of county $$i$$
- $$S_{S,min,i}$$ = system-level lower-bound capacity of county $$i$$
- $$S_{S,max,i}$$ = system-level upper-bound capacity of county $$i$$
- $$S_{D,min,i}$$ = equipment-level lower-bound capacity of county $$i$$
- $$S_{D,max,i}$$ = equipment-level upper-bound capacity of county $$i$$

---

## 9.3 Reallocation Trigger

If:

$$
S_{D,max,i} < S_{S,min,i}
$$

Then the system-level capacity assigned to county $$i$$ is higher than its equipment capability.

In this case, the implementation should generate a warning:

```text
County system-level allocation exceeds equipment-level capability. System capacity reallocation is required.
```

Optionally, implement automatic reallocation.

---

## 9.4 Reconciliation Pseudocode

```python
def reconcile_county_capacity(counties):
    warnings = []

    for c in counties:
        SS_min = c.systemHostingCapacityMinMW
        SS_max = c.systemHostingCapacityMaxMW
        SD_min = c.equipmentHostingCapacityMinMW
        SD_max = c.equipmentHostingCapacityMaxMW

        c.reconciledHostingCapacityMinMW = min(SS_min, SD_min)
        c.reconciledHostingCapacityMaxMW = min(SS_max, SD_max)

        if SD_max < SS_min:
            warnings.append({
                "countyId": c.id,
                "type": "REALLOCATION_REQUIRED",
                "message": "Equipment upper-bound capacity is lower than system lower-bound capacity."
            })

    return counties, warnings
```

---

## 10. Accessible Capacity Evaluation

Accessible capacity shall be evaluated for:

1. County-level administrative areas
2. Transformers at 220/330 kV and below

The calculation must distinguish between:

- Accessible grid-connection capacity
- Accessible registration capacity

---

## 10.1 County-Level Accessible Capacity

For county $$i$$, after reconciliation:

$$
C_{s1,i} = S_i - S_{s,con,i}
$$

$$
C_{s2,i} = C_{s1,i} - S_{s,reg,i}
$$

Where:

| Symbol | Description |
|---|---|
| $$C_{s1,i}$$ | Accessible grid-connection capacity of county $$i$$ |
| $$C_{s2,i}$$ | Accessible registration capacity of county $$i$$ |
| $$S_i$$ | Reconciled hosting capacity of county $$i$$ |
| $$S_{s,con,i}$$ | Already grid-connected distributed-resource capacity in county $$i$$ |
| $$S_{s,reg,i}$$ | Registered but not yet grid-connected distributed-resource capacity in county $$i$$ |

Interval calculation:

$$
C_{s1,min,i} = S_{min,i} - S_{s,con,i}
$$

$$
C_{s1,max,i} = S_{max,i} - S_{s,con,i}
$$

$$
C_{s2,min,i} = C_{s1,min,i} - S_{s,reg,i}
$$

$$
C_{s2,max,i} = C_{s1,max,i} - S_{s,reg,i}
$$

---

## 10.2 Transformer-Level Accessible Capacity

For transformer $$k$$:

$$
C_{d1,k} = S_{d,k} - S_{d,con,k}
$$

$$
C_{d2,k} = C_{d1,k} - S_{d,reg,k}
$$

Where:

| Symbol | Description |
|---|---|
| $$C_{d1,k}$$ | Accessible grid-connection capacity of transformer $$k$$ |
| $$C_{d2,k}$$ | Accessible registration capacity of transformer $$k$$ |
| $$S_{d,k}$$ | Equipment-level hosting capacity of transformer $$k$$ |
| $$S_{d,con,k}$$ | Already grid-connected distributed-resource capacity under transformer $$k$$ |
| $$S_{d,reg,k}$$ | Registered but not yet grid-connected distributed-resource capacity under transformer $$k$$ |

Interval calculation:

$$
C_{d1,min,k} = S_{d,min,k} - S_{d,con,k}
$$

$$
C_{d1,max,k} = S_{d,max,k} - S_{d,con,k}
$$

$$
C_{d2,min,k} = C_{d1,min,k} - S_{d,reg,k}
$$

$$
C_{d2,max,k} = C_{d1,max,k} - S_{d,reg,k}
$$

---

## 10.3 Accessible Capacity Pseudocode

```python
def calculate_county_accessible_capacity(counties):
    for c in counties:
        S_min = c.reconciledHostingCapacityMinMW
        S_max = c.reconciledHostingCapacityMaxMW

        connected = c.existingDistributedResourceMW or 0
        registered = c.registeredDistributedResourceMW or 0

        c.accessibleGridConnectionCapacityMinMW = S_min - connected
        c.accessibleGridConnectionCapacityMaxMW = S_max - connected

        c.accessibleRegistrationCapacityMinMW = (
            c.accessibleGridConnectionCapacityMinMW - registered
        )
        c.accessibleRegistrationCapacityMaxMW = (
            c.accessibleGridConnectionCapacityMaxMW - registered
        )

    return counties
```

---

```python
def calculate_transformer_accessible_capacity(transformers):
    for t in transformers:
        S_min = t.equipmentHostingCapacityMinMW
        S_max = t.equipmentHostingCapacityMaxMW

        connected = t.existingDistributedResourceMW or 0
        registered = t.registeredDistributedResourceMW or 0

        t.accessibleGridConnectionCapacityMinMW = S_min - connected
        t.accessibleGridConnectionCapacityMaxMW = S_max - connected

        t.accessibleRegistrationCapacityMinMW = (
            t.accessibleGridConnectionCapacityMinMW - registered
        )
        t.accessibleRegistrationCapacityMaxMW = (
            t.accessibleGridConnectionCapacityMaxMW - registered
        )

    return transformers
```

---

## 11. Green / Yellow / Red Grade Classification

## 11.1 County-Level Classification

For each county, use the lower-bound values.

Let:

- $$C_{s1,min}$$ = county accessible grid-connection capacity lower bound
- $$C_{s2,min}$$ = county accessible registration capacity lower bound

Classification:

| Grade | Condition | Meaning |
|---|---|---|
| Green | $$C_{s2,min} > 0$$ | After all registered projects are connected, there is still remaining access capacity |
| Yellow | $$C_{s2,min} \leq 0$$ and $$C_{s1,min} > 0$$ | Existing registered projects cannot all be connected; registration warning is required |
| Red | $$C_{s1,min} \leq 0$$ | No additional grid-connection space; suspend new distributed-resource connection before capacity improves |

---

### 11.1.1 County Classification Pseudocode

```python
def classify_counties(counties):
    for c in counties:
        cs1_min = c.accessibleGridConnectionCapacityMinMW
        cs2_min = c.accessibleRegistrationCapacityMinMW

        if cs2_min > 0:
            c.grade = "green"
        elif cs2_min <= 0 and cs1_min > 0:
            c.grade = "yellow"
        elif cs1_min <= 0:
            c.grade = "red"
        else:
            c.grade = "red"

    return counties
```

---

## 11.2 Transformer-Level Classification

Transformer classification must follow the **subordinate-obeys-superior** principle.

For transformer $$k$$:

- Use its own accessible capacity lower-bound values.
- Also check the grade of its superior object.

The superior object may be:

- County
- Upper-voltage transformer
- Parent transformer in the network hierarchy

Let:

- $$C_{d1,min}$$ = transformer accessible grid-connection capacity lower bound
- $$C_{d2,min}$$ = transformer accessible registration capacity lower bound

Classification:

| Grade | Condition |
|---|---|
| Green | $$C_{d2,min} > 0$$ and superior grade is not Red |
| Yellow | $$C_{d2,min} \leq 0$$ and $$C_{d1,min} > 0$$ and superior grade is not Red |
| Red | $$C_{d1,min} \leq 0$$, or superior grade is Red |

---

### 11.2.1 Assessment Order

Transformer-level grading should follow this order:

```text
1. County administrative area
2. 330/220 kV transformer
3. 110/66 kV transformer
4. 35 kV transformer
5. 10 kV transformer
```

---

### 11.2.2 Transformer Classification Pseudocode

```python
def classify_transformers(transformers, counties):
    voltage_order = ["330kV", "220kV", "110kV", "66kV", "35kV", "10kV"]

    transformers_by_id = {t.id: t for t in transformers}
    counties_by_id = {c.id: c for c in counties}

    for voltage in voltage_order:
        for t in [x for x in transformers if x.voltageLevel == voltage]:
            superior_grade = get_superior_grade(t, transformers_by_id, counties_by_id)

            cd1_min = t.accessibleGridConnectionCapacityMinMW
            cd2_min = t.accessibleRegistrationCapacityMinMW

            if superior_grade == "red":
                t.grade = "red"
            elif cd2_min > 0:
                t.grade = "green"
            elif cd2_min <= 0 and cd1_min > 0:
                t.grade = "yellow"
            elif cd1_min <= 0:
                t.grade = "red"
            else:
                t.grade = "red"

    return transformers
```

---

```python
def get_superior_grade(transformer, transformers_by_id, counties_by_id):
    if transformer.parentTransformerId:
        parent = transformers_by_id.get(transformer.parentTransformerId)
        if parent:
            return parent.grade

    county = counties_by_id.get(transformer.countyId)
    if county:
        return county.grade

    return "red"
```

---

## 12. Optional Engineering Verification

You already have mature functions for:

- Power-flow calculation
- Short-circuit calculation
- Harmonic power flow
- Voltage-deviation analysis

The hosting-capacity result should be treated as a planning-level screening result. For actual project connection or detailed access approval, further engineering verification should be performed.

---

## 12.1 Verification Trigger

Engineering verification should be triggered when:

- A new distributed-resource project is proposed
- A county or transformer is Yellow or Red
- Calculated accessible capacity is close to zero
- Large-capacity distributed resources are connected at medium or high voltage
- Network topology changes
- Reverse power flow is significant
- Power quality risk is suspected

---

## 12.2 Power-Flow Verification

For a candidate access scenario, run power-flow calculation to verify:

- Bus voltage within limits
- Transformer loading within limits
- Line loading within limits
- Reverse power flow within allowed range
- Reactive-power balance
- Voltage regulation capability

Recommended scenario construction:

```text
1. Set load to spring/autumn noon low-load condition.
2. Set distributed PV output to maximum simultaneous output.
3. Set existing distributed resources to expected output.
4. Set proposed new distributed resources to rated output multiplied by output coefficient.
5. Set storage operation mode according to assessment scenario.
6. Run AC power flow.
7. Check violations.
```

---

## 12.3 Short-Circuit Verification

Run short-circuit calculation to verify:

- Three-phase short-circuit current
- Single-phase-to-ground short-circuit current
- Breaker interrupting capability
- Bus short-circuit current limit
- Protection-device applicability
- Impact of inverter-based and rotating distributed generators

The project should be rejected or require mitigation if short-circuit current exceeds equipment limits.

---

## 12.4 Voltage-Deviation Verification

Run voltage-deviation analysis according to applicable power-quality requirements.

Check:

- Steady-state voltage deviation
- Voltage rise caused by reverse power flow
- Transformer tap-position feasibility
- Reactive-power compensation interaction
- Distributed-resource power-factor control
- Volt-VAR or Volt-Watt control effect

---

## 12.5 Harmonic Verification

Run harmonic power-flow or harmonic penetration analysis to verify:

- Harmonic voltage distortion
- Harmonic current injection
- Resonance risk
- Interaction with capacitor banks
- Compliance with local harmonic limits
- Aggregated inverter harmonic effects

---

## 12.6 Engineering Verification Result Model

```typescript
interface EngineeringVerificationResult {
  targetType: "county" | "transformer" | "project";
  targetId: string;

  powerFlowPassed: boolean;
  shortCircuitPassed: boolean;
  voltageDeviationPassed: boolean;
  harmonicPassed: boolean;

  violations: EngineeringViolation[];

  finalRecommendation:
    | "allow_connection"
    | "allow_with_mitigation"
    | "suspend_connection"
    | "require_further_study";
}
```

---

```typescript
interface EngineeringViolation {
  type:
    | "voltage_limit"
    | "line_overload"
    | "transformer_overload"
    | "reverse_power_exceedance"
    | "short_circuit_exceedance"
    | "harmonic_exceedance"
    | "protection_issue";

  equipmentId?: string;
  busId?: string;
  phase?: string;

  value: number;
  limit: number;
  unit: string;

  severity: "low" | "medium" | "high" | "critical";

  message: string;
}
```

---

## 13. End-to-End Main Program Flow

```python
def run_distributed_resource_hosting_capacity_assessment(input_data):
    # 1. Load administrative areas and transformers
    counties = input_data.counties
    transformers = input_data.transformers

    # 2. Calculate system-level hosting capacity for province / dispatch area
    if input_data.assessmentMode == "annual":
        system_capacity = calculate_annual_system_capacity_interval(
            input_data.systemSimulationInput
        )
    else:
        system_capacity = calculate_planning_cycle_system_capacity_interval(
            input_data.systemSimulationInput
        )

    # 3. Decompose system-level capacity to counties
    counties = decompose_system_capacity_to_counties(
        counties=counties,
        total_system_capacity_min_mw=system_capacity["systemHostingCapacityMinMW"],
        total_system_capacity_max_mw=system_capacity["systemHostingCapacityMaxMW"],
        weights=input_data.countyAllocationWeights
    )

    # 4. Calculate equipment-level hosting capacity for each transformer
    for t in transformers:
        t = calculate_transformer_hosting_capacity(t)

    # 5. Aggregate equipment-level capacity to county level
    counties = aggregate_equipment_capacity_to_counties(
        counties=counties,
        transformers=transformers,
        supply_ratios=input_data.transformerCountySupplyRatios
    )

    # 6. Reconcile system-level and equipment-level county capacity
    counties, reconciliation_warnings = reconcile_county_capacity(counties)

    # 7. Calculate accessible capacity
    counties = calculate_county_accessible_capacity(counties)
    transformers = calculate_transformer_accessible_capacity(transformers)

    # 8. Classify counties
    counties = classify_counties(counties)

    # 9. Classify transformers according to superior-subordinate principle
    transformers = classify_transformers(transformers, counties)

    # 10. Optional engineering verification
    verification_results = []

    if input_data.enableEngineeringVerification:
        verification_results = run_engineering_verification(
            counties=counties,
            transformers=transformers,
            networkModel=input_data.networkModel,
            candidateProjects=input_data.candidateProjects
        )

    # 11. Return final assessment
    return {
        "systemCapacity": system_capacity,
        "counties": counties,
        "transformers": transformers,
        "warnings": reconciliation_warnings,
        "engineeringVerificationResults": verification_results
    }
```

---

## 14. Recommended Database Tables

## 14.1 `administrative_area`

| Column | Type | Description |
|---|---|---|
| id | varchar | Area ID |
| name | varchar | Area name |
| type | varchar | Province / dispatch area / city / county |
| parent_id | varchar | Parent area ID |
| annual_peak_load_mw | decimal | Annual peak load |
| annual_energy_mwh | decimal | Annual electricity consumption |
| user_count | integer | Number of users |
| gdp | decimal | GDP |
| existing_dr_mw | decimal | Existing grid-connected distributed resources |
| registered_dr_mw | decimal | Registered but not connected distributed resources |
| grade | varchar | Green / Yellow / Red |

---

## 14.2 `transformer`

| Column | Type | Description |
|---|---|---|
| id | varchar | Transformer ID |
| name | varchar | Transformer name |
| voltage_level | varchar | Voltage level |
| parent_transformer_id | varchar | Parent transformer |
| county_id | varchar | County ID |
| rated_capacity_mva | decimal | Rated capacity |
| power_factor | decimal | Power factor |
| operation_mode | varchar | Single / parallel / split |
| max_reverse_load_rate | decimal | Maximum reverse load rate |
| typical_load_mw | decimal | Typical-time load |
| non_dr_generation_mw | decimal | Non-distributed generation output |
| storage_charging_mw | decimal | Existing storage charging power |
| new_storage_charging_min_mw | decimal | Expected new storage lower bound |
| new_storage_charging_max_mw | decimal | Expected new storage upper bound |
| dr_max_output_coefficient | decimal | Maximum output coefficient |
| existing_dr_mw | decimal | Existing DR under transformer |
| registered_dr_mw | decimal | Registered DR under transformer |
| grade | varchar | Green / Yellow / Red |

---

## 14.3 `hosting_capacity_result`

| Column | Type | Description |
|---|---|---|
| id | varchar | Result ID |
| target_type | varchar | County / transformer / system |
| target_id | varchar | Target ID |
| capacity_type | varchar | System / equipment / reconciled / accessible |
| min_mw | decimal | Lower-bound value |
| max_mw | decimal | Upper-bound value |
| assessment_time | datetime | Assessment timestamp |
| assessment_period | varchar | Annual / quarterly / planning-cycle |
| version | varchar | Assessment version |

---

## 14.4 `engineering_verification_result`

| Column | Type | Description |
|---|---|---|
| id | varchar | Result ID |
| target_type | varchar | County / transformer / project |
| target_id | varchar | Target ID |
| power_flow_passed | boolean | Power-flow result |
| short_circuit_passed | boolean | Short-circuit result |
| voltage_deviation_passed | boolean | Voltage-deviation result |
| harmonic_passed | boolean | Harmonic result |
| final_recommendation | varchar | Final engineering recommendation |

---

## 15. Output Report Structure

The system should generate both machine-readable and human-readable outputs.

---

## 15.1 JSON Output

```json
{
  "assessmentId": "HC-2026-Q3-001",
  "assessmentDate": "2026-07-09",
  "assessmentMode": "quarterly",
  "systemCapacity": {
    "minMW": 8000,
    "maxMW": 10000
  },
  "counties": [
    {
      "id": "county_001",
      "name": "Example County",
      "systemCapacityMinMW": 600,
      "systemCapacityMaxMW": 800,
      "equipmentCapacityMinMW": 730,
      "equipmentCapacityMaxMW": 890,
      "reconciledCapacityMinMW": 600,
      "reconciledCapacityMaxMW": 800,
      "accessibleGridConnectionCapacityMinMW": 100,
      "accessibleGridConnectionCapacityMaxMW": 300,
      "accessibleRegistrationCapacityMinMW": -50,
      "accessibleRegistrationCapacityMaxMW": 150,
      "grade": "yellow"
    }
  ],
  "transformers": [
    {
      "id": "tr_001",
      "name": "110kV Transformer 1",
      "voltageLevel": "110kV",
      "equipmentCapacityMinMW": 142.35,
      "equipmentCapacityMaxMW": 165.88,
      "accessibleGridConnectionCapacityMinMW": 22.35,
      "accessibleGridConnectionCapacityMaxMW": 45.88,
      "accessibleRegistrationCapacityMinMW": -7.65,
      "accessibleRegistrationCapacityMaxMW": 15.88,
      "grade": "yellow"
    }
  ],
  "warnings": [
    {
      "countyId": "county_002",
      "type": "REALLOCATION_REQUIRED",
      "message": "Equipment upper-bound capacity is lower than system lower-bound capacity."
    }
  ]
}
```

---

## 15.2 Markdown / PDF Report

Recommended report sections:

```text
1. Assessment Overview
2. Input Data Summary
3. System-Level Hosting Capacity Result
4. County-Level System Capacity Decomposition
5. Equipment-Level Hosting Capacity Result
6. County-Level Reconciliation Result
7. Accessible Capacity Result
8. Green / Yellow / Red Grade Map
9. Warning List
10. Engineering Verification Result
11. Capacity Improvement Recommendations
```

---

## 16. Capacity Improvement Recommendations

When Yellow or Red areas are identified, the system should generate recommended improvement measures.

---

## 16.1 System-Level Improvement Measures

Recommended measures include:

- Optimize renewable-energy utilization target under policy constraints
- Increase load development
- Build pumped-storage stations
- Add new-energy storage
- Retrofit thermal units for flexibility
- Improve inter-provincial and inter-regional transmission capability
- Improve demand-response participation
- Optimize market-based dispatch

---

## 16.2 Equipment-Level Improvement Measures

Recommended measures include:

- Add new-energy storage under constrained transformers
- Use collection and step-up connection schemes
- Reconfigure feeders
- Expand transformer capacity
- Add distributed-resource tripping or security-control devices
- Optimize transformer operation mode
- Improve reactive-power compensation
- Add voltage-control equipment
- Adjust access points of distributed resources

---

## 17. Validation Rules

The implementation should include strong input validation.

---

## 17.1 Required Validation

```python
def validate_transformer_input(t):
    assert t.ratedCapacityMVA > 0
    assert 0 < t.powerFactor <= 1
    assert 0 < t.maxReverseLoadRate <= 1.5
    assert 0 < t.distributedResourceMaxOutputCoefficient <= 1.2
    assert t.expectedNewStorageChargingPowerMinMW <= t.expectedNewStorageChargingPowerMaxMW
```

---

## 17.2 Capacity Interval Validation

```python
def validate_capacity_interval(min_value, max_value):
    if min_value > max_value:
        raise ValueError("Capacity interval lower bound exceeds upper bound.")
```

---

## 17.3 County Weight Validation

```python
def validate_weights(weights):
    total = sum(weights.values())
    if abs(total - 1.0) > 0.0001:
        raise ValueError("County allocation weights must sum to 1.")
```

---

## 18. Implementation Notes

### 18.1 Unit Consistency

Use MW and MVA consistently.

Recommended:

| Quantity | Unit |
|---|---|
| Active power | MW |
| Apparent power | MVA |
| Energy | MWh |
| Capacity | MW |
| Utilization rate | Per unit |
| Power factor | Per unit |
| Reverse load rate | Per unit |

When using transformer rated capacity:

$$
P_{rated} \approx S_{rated} \times cos\theta
$$

---

### 18.2 Negative Accessible Capacity

Accessible capacity may be negative.

Do not forcibly clamp accessible capacity to zero, because negative values indicate overload or over-registration risk.

For example:

$$
C_{s2,min} = -50MW
$$

Means registered-but-not-connected projects exceed the conservative remaining registration capacity by 50 MW.

---

### 18.3 Hosting Capacity Should Be Non-Negative

Hosting capacity itself should not be negative.

If equipment-level formula produces a negative result, clamp hosting capacity to zero:

$$
S_d = max(S_d, 0)
$$

---

### 18.4 Grade Classification Uses Lower Bound

Always use lower-bound accessible capacity for grading:

- $$C_{s1,min}$$
- $$C_{s2,min}$$
- $$C_{d1,min}$$
- $$C_{d2,min}$$

The lower bound represents the conservative scenario.

---

### 18.5 Registered Capacity Validity

Only registered projects within the valid registration period should be included in:

$$
S_{s,reg}
$$

and:

$$
S_{d,reg}
$$

Expired registrations should be excluded.

---

## 19. Minimum Viable Product Implementation

If implementing an MVP, prioritize the following modules:

```text
1. Transformer data import
2. Equipment-level hosting capacity calculation
3. Transformer accessible capacity calculation
4. Transformer Green / Yellow / Red classification
5. County aggregation
6. County accessible capacity calculation
7. County Green / Yellow / Red classification
8. Report generation
```

Then add:

```text
9. System-level production simulation
10. County-level system-capacity decomposition
11. Reconciliation between system and equipment capacity
12. Engineering verification integration
```

---

## 20. Final Deliverables

The implementation should produce:

1. **System-level hosting capacity interval**
2. **County-level system hosting capacity interval**
3. **Transformer-level equipment hosting capacity interval**
4. **County-level equipment hosting capacity interval**
5. **County-level reconciled hosting capacity interval**
6. **Accessible grid-connection capacity interval**
7. **Accessible registration capacity interval**
8. **Green / Yellow / Red assessment grade**
9. **Warning list**
10. **Optional engineering-verification results**
11. **Capacity-improvement recommendations**

This specification is sufficient to implement a reproducible distributed-resource hosting-capacity assessment engine compatible with the DL/T 2041—2025 assessment logic.
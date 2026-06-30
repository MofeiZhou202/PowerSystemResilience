# Reliability Assessment Code Review and Unification Design

Date: 2026-06-30

Repository: `HybridACDCDistributionSystemsSimulation`

Scope: reliability assessment code, reliability data contracts, model-scope consistency, metrics, GUI/API exposure, and a proposed design path for unifying the four currently available reliability methods.

Current methods reviewed:

1. Non-sequential Monte Carlo reliability assessment.
2. Sequential Monte Carlo reliability assessment.
3. Distribution FMEA / N-1 failure-mode enumeration.
4. Frequency and duration analytical assessment.

Related reliability/restoration method:

5. Three-stage fault-recovery reliability evaluator. This is not currently exposed in the same web reliability selector, but it should be part of the future reliability model choices because it is a distinct restoration/reconfiguration model.

## Executive Summary

The reliability assessment code has grown into several parallel implementations with different assumptions about component reliability parameters, component scope, physics scope, and metrics. The most important issue is not a single solver bug. The main issue is that the four methods do not share one reliability data contract.

At present:

- `failure_rate` usually means failures/year for AC branches.
- `forced_outage_rate` usually means steady-state unavailability or FOR.
- Several rich components use `mtbf_hours` and `mttr_hours`.
- Some methods convert these fields to unavailability, while others convert them to failure frequency.
- Missing data is often replaced by hard-coded defaults.
- GUI endpoints can automatically apply templates that overwrite case-specific reliability data.
- Model limitations already exist in core result structs, but the GUI API strips many of them.

This can produce reliability results that look authoritative while mixing different meanings of "rate", "outage probability", "repair time", "curtailment", and "customer interruption".

Recommended first milestone:

1. Define one reliability parameter contract.
2. Preserve all reliability fields through JSON/API/GUI.
3. Surface model scope and validity in every result.
4. Fix hybrid AC/DC customer metrics.
5. Make reliability template data opt-in.
6. Then unify method-specific engines behind a common assessment request/result schema.

## Current Code Map

### Core Reliability Engine

Main files:

- `include/hacdcpf/reliability/reliability_assessment.hpp`
- `src/reliability/reliability_assessment.cpp`
- `src/reliability/reliability_data.cpp`

Implemented methods:

- `run_nonsequential_mc`
- `run_sequential_mc`
- `run_distribution_fmea`
- `run_frequency_duration_analysis`
- `compute_distribution_indices`
- `apply_ieee24_reliability_data`
- `apply_comprehensive_reliability_data`

Important current model-scope declarations:

- `ReliabilityResult::model_scope`
- `ReliabilityResult::model_limitations`
- `ReliabilityResult::validity`
- `FMEAResult::model_scope`
- `FMEAResult::model_limitations`
- `FMEAResult::validity`

These declarations are good and should become mandatory API output fields.

### Three-Stage Reliability Engine

Main files:

- `include/hacdcpf/analysis/three_stage_reliability.hpp`
- `src/reliability/three_stage_reliability.cpp`

Implemented method:

- `run_three_stage_reliability`
- `run_three_stage_reliability_from_string`

This evaluator solves a staged restoration model:

- Stage 1: fault isolation.
- Stage 2: post-fault switching restoration.
- Stage 3: post-repair topology restoration.

The current implementation uses an AC LinDistFlow MILP plus a DC connectivity/capacity fallback for hybrid networks.

### GUI and API

Main files:

- `tests/run_gui_server.cpp`
- `web/index.html`
- `web/js/app.js`

Current web selector exposes:

- `nsq`
- `seq`
- `fmea`
- `fd`

Three-stage reliability is not part of the same GUI selector yet.

### JSON I/O

Main file:

- `src/io/json_io.cpp`

This layer is critical for reliability because GUI editing, scenario import/export, and round-trip case persistence all depend on it.

### Canonical and Rich Model Boundary

Main files:

- `include/hacdcpf/model/effective_capacity.hpp`
- `include/hacdcpf/projection/project_to_canonical.hpp`
- `include/hacdcpf/projection/canonical_network.hpp`
- `src/model/network_utils.cpp`

Reliability code currently mixes direct rich-component handling with solver paths that often use canonicalized network models. This needs an explicit policy.

## Current Method Behavior

### 1. Non-Sequential Monte Carlo

Entry point:

- `run_nonsequential_mc`

Current physical evaluator:

- Calls `evaluate_state`.
- Uses AC-only DC-OPF.
- Samples a broad state vector containing AC generators, AC branches, static generators, renewable generators, AC storage, VSC converters, DC branches, transformers, DC/DC converters, DCCBs, DC storage, DC PV, AC switches, AC circuit breakers, AC PV systems, and DC static generators.

Current limitation:

- DC/VSC/rich components may be sampled, but DC load curtailment and DC power-flow balance are not included in EENS/LOLE.
- Failures of DC components can affect flags and in-service status, but the physical consequence is not a full hybrid AC/DC OPF.

Current result object already says this:

- `model_scope = "ac-only-dcopf"`
- `validity.dc_load_curtailment_included = false`
- `validity.vsc_dc_power_flow_modelled = false`

Issue:

- GUI endpoint does not return these fields.

### 2. Sequential Monte Carlo

Entry point:

- `run_sequential_mc`

Current physical evaluator:

- Also calls `evaluate_state`.
- Uses AC-only DC-OPF.
- Uses chronological up/down sampling with MTTF/MTTR derived per component.

Current limitation:

- Same physical limitation as non-sequential MC.
- DC load curtailment is excluded from system EENS/LOLE.
- Method-specific defaults for MTTF/MTTR differ from non-sequential MC default unavailabilities.

Current result object already says this:

- `model_scope = "ac-only-dcopf"`
- validity flags identify missing DC/VSC coverage.

Issue:

- GUI endpoint does not return these fields.

### 3. Distribution FMEA / N-1 Enumeration

Entry point:

- `run_distribution_fmea`

Current physical evaluator:

- Builds a component catalog.
- Evaluates switching stage and repair stage for each N-1 component outage.
- For hybrid systems, uses `evaluate_hybrid_fmea_network_lp`.
- For AC-only systems, falls back to AC-only DC-OPF.

Hybrid LP scope:

- AC branch transfer limits.
- DC branch transfer limits.
- AC/DC load shedding.
- DC sources.
- DC/DC active transfer.
- VSC active-power transfer.
- Storage emergency support when enabled.

Current limitations:

- No nonlinear AC power-flow validation.
- No reactive feasibility certification.
- No AC voltage feasibility in the hybrid LP.
- Repair-stage explicit reconfiguration enumerates AC switch/branch actions only.
- DC breaker, DC branch, VSC, and DCDC topology actions are not considered as repair reconfiguration candidates.
- Direct rich-component treatment can diverge from canonical solver projection.

Result object already exposes:

- `model_scope`
- `model_limitations`
- `validity`
- per-contingency stage fields.

Issues:

- GUI strips many important fields.
- GUI calculates `n_with_loss` using only repair-stage shed, ignoring switching-stage-only loss.
- Hybrid SAIFI/SAIDI do not currently include DC customers correctly.

### 4. Frequency and Duration Analytical Method

Entry point:

- `run_frequency_duration_analysis`

Current scope:

- Generation adequacy only.
- AC generators only.
- Constant peak load.
- No network contingencies.
- No DC network.
- No chronological load profile.
- No load-point/customer interruption metrics.

This method is still useful, but it should be labeled as "generation adequacy F&D", not "full system reliability".

### 5. Three-Stage Fault-Recovery Reliability

Entry point:

- `run_three_stage_reliability`

Current scope:

- Enumerates in-service ACBranch and DCBranch outages.
- AC network is evaluated with a LinDistFlow restoration MILP.
- DC network uses a connectivity/capacity fallback.
- VSC/DCDC are graph edges in the fallback, not fully dispatched physical devices.

Current limitations:

- Branch outages only.
- Generator, load, transformer, switch, breaker, VSC, DCDC, storage, and PV outages are not enumerated.
- Stage durations are hard-coded constants in the implementation.
- Per-component MTTR is not currently used as repair-stage duration in the reported reliability indices.
- Hybrid cases return metrics with `ok == false` because the DC/VSC/SOP portion is fallback-only.

This method should be exposed in the GUI as a separate model choice, but with strong model-scope badges.

## Critical Findings

### Finding 1: Reliability Parameter Semantics Are Not Unified

Severity: Critical

Current fields have different meanings by component and method:

- ACBranch: `failure_rate` is failures/year.
- Generator: `forced_outage_rate` is FOR/unavailability.
- StaticGenerator, RenewableGen, DCBranch, DCDC, transformers: `mtbf_hours` and `mttr_hours`.
- Storage and VSC often use `forced_outage_rate` plus repair time.
- Switches have `p_sw_fail`, `mtbf_hours`, and `mttr_hours`.
- Some breakers have no reliability fields.

Current conversions:

- Non-sequential MC needs unavailability.
- Sequential MC needs MTTF/MTTR.
- FMEA needs failure frequency and repair duration.
- F&D needs generator availability and transition frequency.

Problem:

The conversions are implemented independently, with different default values. That means the same case can produce inconsistent component risk between methods.

Required design:

Create a single resolver:

```cpp
struct ReliabilityParams {
  bool has_data{false};
  bool used_default{false};
  std::string data_source;
  double lambda_per_year{0.0};
  double repair_hr{0.0};
  double unavailability{0.0};
  double mttf_hr{0.0};
  std::vector<std::string> warnings;
};
```

Every method should call one resolver:

```cpp
ReliabilityParams resolve_reliability_params(
    const HybridPowerSystem& sys,
    ComponentRef component,
    const ReliabilityDataPolicy& policy);
```

Rules:

- If lambda/year and repair hours are provided:
  - `unavailability = lambda / (lambda + 8760 / repair_hr)`
  - `mttf_hr = 8760 / lambda`
- If MTBF and MTTR are provided:
  - `lambda_per_year = 8760 / mtbf_hours`
  - `repair_hr = mttr_hours`
  - `unavailability = mttr / (mtbf + mttr)`
- If FOR and MTTR are provided:
  - `unavailability = FOR`
  - `lambda_per_year = FOR / ((1 - FOR) * MTTR) * 8760`
  - `mttf_hr = MTTR * (1 - FOR) / FOR`
- If data is missing:
  - Do not invent values unless the user selects a named default library.

### Finding 2: Missing Data Is Silently Replaced With Defaults

Severity: Critical

Examples:

- Non-sequential MC uses default unavailability values when fields are missing.
- Sequential MC uses default MTTF/MTTR values when fields are missing.
- FMEA uses default lambda and repair time values when fields are missing.
- GUI endpoints default to applying IEEE-24 or comprehensive reliability data.

Problem:

The user cannot tell whether results came from the case data or from hidden defaults.

Required design:

Introduce `ReliabilityDataPolicy`:

```cpp
enum class ReliabilityDefaultPolicy {
  StrictCaseDataOnly,
  UseNamedTemplateForMissingOnly,
  OverwriteWithNamedTemplate
};

struct ReliabilityDataPolicy {
  ReliabilityDefaultPolicy default_policy{ReliabilityDefaultPolicy::StrictCaseDataOnly};
  std::string template_name;
  bool fail_on_missing_required_data{true};
  bool report_defaulted_components{true};
};
```

GUI default should be `StrictCaseDataOnly`.

Named templates should be explicit:

- IEEE RTS-24.
- Comprehensive hybrid distribution demo.
- Utility typical distribution.
- User uploaded reliability library.

### Finding 3: Hybrid Customer Metrics Are Incomplete

Severity: Critical

FMEA hybrid evaluation can shed DC load and include it in EENS/LOLE. But `compute_distribution_indices()` only builds customer weights from AC buses and AC loads. DC load interruption frequencies and durations can be present in the nodal vectors but excluded from SAIFI/SAIDI weighting.

Required design:

Add a hybrid load-point metric builder:

```cpp
struct ReliabilityLoadPoint {
  enum class Domain { AC, DC };
  Domain domain;
  int bus_id;
  size_t nodal_index;
  double demand_mw;
  double customers;
  double priority_weight;
};
```

Then compute:

- SAIFI = sum(lambda_i * customers_i) / total_customers
- SAIDI = sum(lambda_i * duration_i * customers_i) / total_customers
- CAIDI = SAIDI / SAIFI
- ASAI = 1 - SAIDI / hours_per_year

This should include:

- AC loads.
- AC bus-level loads.
- DC loads.
- DC bus-level loads.

### Finding 4: GUI/API Hides Model Scope and Validity

Severity: High

Core result structs contain model-scope and validity flags, but GUI endpoints omit them.

Required API output for every reliability method:

```json
{
  "method": "fmea",
  "model_scope": "hybrid-acdc-network-lp",
  "model_limitations": "...",
  "validity": {
    "dc_load_curtailment_included": true,
    "vsc_dc_power_flow_modelled": true,
    "ac_opf_curtailment": true,
    "ac_voltage_reactive_feasibility_certified": false
  },
  "data_quality": {
    "components_total": 0,
    "components_with_reliability_data": 0,
    "components_defaulted": 0,
    "missing_required_data": []
  },
  "metrics": {}
}
```

The GUI should always show:

- Method.
- Physical model.
- Reliability data policy.
- Scope badge: exact, approximate, fallback, or adequacy-only.
- Missing/defaulted data warnings.
- Whether DC load curtailment is included.
- Whether AC voltage/reactive feasibility is certified.

### Finding 5: FMEA Rich Model May Diverge From Canonical Solver Model

Severity: High

Canonical projection expands rich components:

- Transformers to equivalent branches.
- Switches to equivalent branches and possible bus merges.
- Circuit breakers to equivalent branches.
- Energy routers to VSC/DCDC structures.
- VPP/microgrid/mobile storage to canonical sources/storage.

FMEA directly models rich components. This can create divergence between FMEA component IDs, branch availability, and actual PF/OPF solver behavior.

Required design decision:

Choose one of these policies:

Policy A: Canonical-first reliability.

- Project the system to canonical model first.
- Evaluate reliability on canonical components.
- Use provenance maps to report original rich components.
- Best for consistency with PF/OPF.

Policy B: Rich-component reliability.

- Reliability owns rich component semantics.
- PF/OPF is used only as a lower-level physical evaluator.
- Requires a rich-to-physical consequence map for every component.
- Best for asset-management interpretation.

Recommendation:

Use a hybrid of both:

- Build the contingency catalog in rich component space.
- Resolve each contingency into a canonical physical model mutation.
- Store provenance both ways:
  - `source_component_ref`
  - `canonical_affected_elements`
  - `modelled_consequence`

### Finding 6: Three-Stage Reliability Uses Fixed Durations for Reliability Indices

Severity: High

The three-stage evaluator uses constants:

- `kTauSwitchHr`
- `kTauTrippingHr`
- `kTauRepairHr`

Problem:

This is suitable for a staged restoration demonstration, but not for component-specific reliability indices if repair times vary by asset.

Required design:

For each faulted component:

- `tau_isolation_hr`
- `tau_switching_hr`
- `tau_repair_hr`

Then:

- Stage 1 duration = isolation time.
- Stage 2 duration = switching/restoration time window.
- Stage 3 duration = repair duration minus previous stages, or a separate post-repair operating interval if that is the intended theory.

Use the resolved component `repair_hr` as the default repair duration.

### Finding 7: F&D Method Is Adequacy-Only

Severity: Medium

The F&D method only builds a generator capacity outage probability table. It does not model network outages, DC equipment, branch constraints, load-point interruption, or restoration.

Required design:

In GUI and API, label this as:

`Generation adequacy frequency-duration`

Not:

`Full reliability assessment`

## Proposed Unified Reliability Architecture

### Layer 1: Component Identity

Introduce a stable component reference:

```cpp
enum class ReliabilityComponentKind {
  ACGenerator,
  ACBranch,
  ACLoad,
  ACBusLoad,
  ACStaticGenerator,
  ACRenewableGenerator,
  ACStorage,
  ACPVSystem,
  ACTransformer2W,
  ACTransformer3W,
  ACSwitch,
  ACCircuitBreaker,
  DCBusLoad,
  DCBranch,
  DCLoad,
  DCStaticGenerator,
  DCDCConverter,
  DCCircuitBreaker,
  DCStorage,
  DCPVArray,
  VSCConverter,
  EnergyRouter,
  Microgrid,
  MobileStorage,
  VirtualPowerPlant
};

struct ComponentRef {
  ReliabilityComponentKind kind;
  int element_index;
  std::string element_name;
  std::string domain;      // "AC", "DC", "Hybrid"
  std::string stable_id;   // for GUI and exported results
};
```

The GUI should use `stable_id`, not only vector position.

### Layer 2: Reliability Parameter Resolver

All methods call the same resolver. It returns:

- lambda/year.
- repair hours.
- unavailability.
- data source.
- default status.
- validation warnings.

This resolver should be unit-tested independently.

### Layer 3: Contingency Catalog

Build one reusable catalog:

```cpp
struct ReliabilityContingency {
  ComponentRef component;
  ReliabilityParams params;
  std::string failure_mode;
  bool enabled;
  std::string reason_disabled;
};
```

The catalog should support filters:

- N-1 components.
- Branch-only.
- Generation-only.
- AC-only.
- DC-only.
- User selected components.
- Components with complete data only.

### Layer 4: Physical Evaluation Model

Separate reliability sampling/enumeration from physical consequence evaluation.

Proposed evaluator interface:

```cpp
enum class ReliabilityPhysicalModel {
  ACOnlyDCOPF,
  HybridNetworkLP,
  ACLinDistFlowRestorationMILP,
  GenerationAdequacyCOPT,
  FutureFullHybridACDCOPF
};

struct ReliabilityPhysicalModelResult {
  double total_shed_mw;
  std::vector<double> nodal_shed_mw;
  std::string model_scope;
  std::string model_limitations;
  nlohmann::json validity;
};
```

Then methods become:

- Non-sequential MC = stochastic state sampling + selected physical evaluator.
- Sequential MC = chronological state sampling + selected physical evaluator.
- FMEA = deterministic N-1 catalog enumeration + selected physical evaluator.
- F&D = analytical generation outage table + adequacy model.
- Three-stage = branch outage catalog + restoration MILP evaluator.

### Layer 5: Unified Result Contract

Every method should return:

```cpp
struct UnifiedReliabilityResult {
  std::string method;
  std::string physical_model;
  std::string data_policy;
  std::string model_scope;
  std::string model_limitations;
  nlohmann::json validity;
  ReliabilityDataQuality data_quality;
  ReliabilityMetrics metrics;
  std::vector<NodalReliabilityMetric> nodal_metrics;
  std::vector<ContingencyReliabilityResult> contingencies;
  std::vector<ComponentImportanceResult> critical_components;
  std::vector<std::string> warnings;
};
```

Minimum common metrics:

- EENS.
- EDNS.
- LOLE.
- LOLF.
- PLC.
- SAIFI.
- SAIDI.
- CAIDI.
- ASAI.

For methods where a metric is not meaningful, return:

```json
{
  "value": null,
  "available": false,
  "reason": "F&D generation adequacy method does not compute customer interruption indices."
}
```

This is better than showing zero.

## GUI Redesign Proposal

### Design Goal

The reliability GUI should make the user choose:

1. Reliability method.
2. Physical consequence model.
3. Reliability data source/policy.
4. Component scope.
5. System analysis options.
6. Output metrics and diagnostics.

It should not silently choose templates or hide model limitations.

### Proposed Reliability Page Layout

Use one main analysis workspace, not separate hidden forms per method.

Sections:

1. Method and physical model.
2. Reliability data.
3. Component scope.
4. Load and time settings.
5. Restoration and switching settings.
6. Advanced solver settings.
7. Run and diagnostics.
8. Results.

### Section 1: Method and Physical Model

Controls:

- Method segmented control:
  - Non-sequential MC.
  - Sequential MC.
  - FMEA N-1.
  - Frequency-duration adequacy.
  - Three-stage restoration.

- Physical model selector:
  - AC-only DC-OPF.
  - Hybrid AC/DC network LP.
  - AC LinDistFlow restoration MILP.
  - Generation adequacy COPT.
  - Future full hybrid AC/DC OPF.

Compatibility behavior:

- F&D only allows generation adequacy COPT.
- Three-stage defaults to AC LinDistFlow restoration MILP.
- MC methods can initially use AC-only DC-OPF, with hybrid LP as a future option.
- FMEA can use AC-only DC-OPF or hybrid AC/DC network LP.

Display badges:

- `AC only`
- `Hybrid included`
- `DC fallback`
- `Voltage not certified`
- `Adequacy only`
- `Restoration MILP`

### Section 2: Reliability Data

Controls:

- Data policy menu:
  - Case data only.
  - Template for missing fields only.
  - Overwrite with template.

- Template menu:
  - None.
  - IEEE RTS-24.
  - Comprehensive hybrid distribution demo.
  - User uploaded library.

- Missing-data behavior:
  - Block run.
  - Warn and skip missing components.
  - Warn and use selected template.

- Component parameter table:
  - Component.
  - Domain.
  - Type.
  - Lambda/year.
  - FOR/unavailability.
  - MTBF hours.
  - MTTR hours.
  - Customers.
  - Source: case/template/default/missing.
  - Validation status.

Table actions:

- Edit selected component.
- Bulk apply template to selected components.
- Export reliability data.
- Import reliability data CSV/JSON.
- Reset edited values.

### Section 3: Component Scope

Controls:

- Include component kinds:
  - AC branches.
  - DC branches.
  - AC generators.
  - DC generators.
  - VSC converters.
  - DCDC converters.
  - Transformers.
  - Switches.
  - AC circuit breakers.
  - DC circuit breakers.
  - Storage.
  - PV.
  - Loads.

- Fault set:
  - N-1 only.
  - Selected components only.
  - Branch-only.
  - Generation-only.
  - Critical components from previous run.

- Domain:
  - AC.
  - DC.
  - Hybrid.

### Section 4: Load and Time Settings

Controls:

- Load scale factor.
- Load profile:
  - Constant.
  - IEEE RTS-24 hourly profile.
  - Uploaded hourly profile.
  - Existing time-series profile.

- Hours per year:
  - 8760.
  - 8736.
  - Custom.

- Customer source:
  - Load table `n_customers`.
  - Bus table `n_customers`.
  - Estimate from MW.
  - Uploaded customer table.

### Section 5: Restoration and Switching Settings

Shown for FMEA and three-stage.

Controls:

- Switching time.
- Isolation time.
- Use component MTTR for repair duration.
- Manual repair duration override.
- Enable repair reconfiguration.
- Enable switch reconfiguration.
- Maximum switch actions.
- Maximum OPF/MILP evaluations.
- Include AC switches.
- Include DC breaker actions.
- Include normally-open branch candidates.

Current code supports only part of this, so unsupported options should be disabled with a clear model-scope message.

### Section 6: Advanced Solver Settings

Controls:

- Random seed.
- Max samples / years.
- CoV threshold.
- Tail risk enabled.
- VaR confidence.
- OPF load shedding penalty.
- MILP gap tolerance.
- MILP time limit.
- Solver log verbosity.

### Section 7: Pre-Run Diagnostics

Before run, show:

- Number of components in scope.
- Number with complete reliability data.
- Number missing reliability data.
- Number defaulted/template-filled.
- Whether DC load curtailment will be included.
- Whether AC voltage/reactive feasibility is certified.
- Whether result will be exact, approximate, or fallback.

Block run only if:

- Required reliability data is missing and policy is strict.
- Selected method/model combination is unsupported.
- No valid contingency/sampling components exist.

### Section 8: Results Design

Result tabs:

1. Summary.
2. Model scope.
3. Data quality.
4. Critical components.
5. Contingencies.
6. Nodal/customer metrics.
7. Convergence and uncertainty.
8. Export.

Summary KPIs:

- EENS.
- EDNS.
- LOLE.
- LOLF.
- PLC.
- SAIFI.
- SAIDI.
- CAIDI.
- ASAI.
- Cost of EENS.

Each KPI should show availability:

- Available.
- Not applicable.
- Approximate.
- Excluded by model.

Model scope tab:

- Model name.
- Validity flags.
- Limitations.
- Physical constraints included/excluded.
- DC treatment.
- VSC/DCDC treatment.
- Restoration treatment.

Contingency table:

- Rank.
- Component name.
- Component kind.
- Domain.
- Failure rate.
- Repair time.
- Switching-stage shed.
- Repair-stage shed.
- EENS contribution.
- LOLE contribution.
- Loss flags.
- Selected repair actions.
- Search truncated flag.

Nodal/customer table:

- Domain.
- Bus/load id.
- Demand.
- Customers.
- EENS.
- CIF.
- CID.
- SAIFI contribution.
- SAIDI contribution.

## API Redesign Proposal

### New Endpoint

Instead of separate method endpoints only, add:

`POST /api/session/run_reliability`

Request:

```json
{
  "method": "fmea",
  "physical_model": "hybrid_network_lp",
  "data_policy": {
    "mode": "case_data_only",
    "template": null,
    "fail_on_missing_required_data": true
  },
  "component_scope": {
    "domains": ["AC", "DC"],
    "kinds": ["ACBranch", "DCBranch", "VSCConverter", "DCDCConverter"],
    "only_in_service": true
  },
  "load": {
    "scale_factor": 1.0,
    "profile": "constant",
    "hours_per_year": 8760
  },
  "restoration": {
    "switching_time_hr": 0.5,
    "use_component_repair_time": true,
    "enable_switch_reconfiguration": true,
    "max_switch_actions": 2,
    "max_physical_evaluations": 200
  },
  "monte_carlo": {
    "max_iterations": 5000,
    "cov_threshold": 0.05,
    "seed": 0,
    "compute_tail_risk": false,
    "var_confidence": 0.95
  }
}
```

Response:

```json
{
  "method": "fmea",
  "physical_model": "hybrid_network_lp",
  "model_scope": "hybrid-acdc-network-lp",
  "model_limitations": "...",
  "validity": {},
  "data_quality": {},
  "metrics": {},
  "contingencies": [],
  "nodal_metrics": [],
  "critical_components": [],
  "warnings": []
}
```

Keep old endpoints temporarily as wrappers:

- `/api/session/run_reliability_nsq`
- `/api/session/run_reliability_seq`
- `/api/session/run_reliability_fmea`
- `/api/session/run_reliability_fd`

But make them call the unified request path internally.

## Debug and Implementation Roadmap

### Phase 0: Preserve Current Behavior But Expose Truth

Goal:

No major solver/model changes. Make outputs honest.

Tasks:

1. Return `model_scope`, `model_limitations`, and `validity` from NSQ/SEQ/FMEA/F&D endpoints.
2. Return complete critical component metadata:
   - component type.
   - component name.
   - global state index.
   - domain.
3. Return complete FMEA contingency details:
   - switching-stage shed and ENS.
   - repair-stage shed and ENS.
   - loss flags.
   - repair switch actions.
   - truncation flag.
   - nodal shedding.
4. Fix GUI `n_with_loss` to use core `n_loss_contingencies` or count both switching and repair stages.
5. Make GUI default reliability template application opt-in.
6. Add visible model-scope warnings in the results page.

Expected result:

Existing results may not change numerically, but users can see what the numbers mean.

### Phase 1: Reliability Parameter Contract

Goal:

All four methods use identical reliability parameters for identical components.

Tasks:

1. Implement `ComponentRef`.
2. Implement `ReliabilityParams`.
3. Implement `ReliabilityDataPolicy`.
4. Implement one resolver for all component types.
5. Replace method-local conversion/default logic with resolver calls.
6. Add data-quality summary.
7. Add strict missing-data mode.

Expected result:

MC, FMEA, and three-stage use the same lambda and repair time for the same component.

### Phase 2: JSON and GUI Data Preservation

Goal:

Reliability data survives round trips.

Tasks:

1. Add missing DC bus fields to JSON:
   - `n_customers`
   - `importance`
   - `is_load`
2. Add missing DC branch reliability fields:
   - `mtbf_hours`
   - `mttr_hours`
   - `t_scheduled_hr`
3. Add missing DC load fields:
   - `scaling`
   - `n_customers`
   - `priority`
   - rated/metadata fields if present in the model.
4. Add reliability fields to breaker models if asset-level reliability is required.
5. Add GUI reliability parameter table.
6. Add import/export of reliability parameter library.

Expected result:

User-entered reliability parameters are not erased by save/load/import/export.

### Phase 3: Hybrid Metrics

Goal:

AC and DC customer metrics are correct.

Tasks:

1. Replace or overload `compute_distribution_indices`.
2. Build explicit AC/DC reliability load points.
3. Include DC loads and DC bus-level loads in SAIFI/SAIDI.
4. Add nodal metrics with domain and bus id.
5. Add tests for DC-only and mixed AC/DC systems.

Expected result:

DC interruption contributes to SAIFI/SAIDI/ASAI, not only EENS/LOLE.

### Phase 4: Method Unification

Goal:

One request/result contract drives all methods.

Tasks:

1. Implement unified reliability request.
2. Implement unified result schema.
3. Add method wrappers.
4. Add GUI method/model compatibility logic.
5. Keep existing endpoints as backward-compatible wrappers.

Expected result:

Adding new reliability methods no longer requires custom GUI/API/result logic.

### Phase 5: Physical Model Alignment

Goal:

Model choices are explicit and consistent with canonical/rich component semantics.

Tasks:

1. Decide canonical-first vs rich-first contingency semantics.
2. Add component provenance to reliability results.
3. Optionally allow FMEA to evaluate canonical-projected consequences.
4. Optionally upgrade MC to use hybrid network LP for AC/DC load curtailment.
5. Extend three-stage durations to component-specific repair times.
6. Extend three-stage contingency catalog beyond branch-only if required.

Expected result:

Reliability results can be compared across methods because they share the same data and clearly declare physical differences.

## Recommended Test Plan

### Unit Tests

Reliability parameter resolver:

- Lambda/year plus MTTR produces correct unavailability.
- MTBF/MTTR produces correct lambda and unavailability.
- FOR/MTTR produces correct lambda and MTTF.
- Missing data in strict mode reports missing, not defaults.
- Template-for-missing mode marks defaulted components.
- Overwrite-template mode marks overwritten components.

JSON round-trip:

- DC bus customer fields survive.
- DC branch MTBF/MTTR survive.
- DC load scaling/customers survive.
- DCDC MTBF/MTTR survive.
- Switch MTBF/MTTR and operation fields survive.

Metrics:

- DC-only load interruption changes SAIFI/SAIDI.
- AC+DC mixed load interruption weights customers correctly.
- ASAI uses configured hours per year.
- FMEA switching-only interruption counts as loss contingency.

### Integration Tests

Non-sequential MC:

- Hybrid system returns `model_scope = ac-only-dcopf`.
- DC load curtailment validity flag is false.
- Missing data strict mode blocks or warns.

Sequential MC:

- Uses same resolved parameters as NSQ.
- Annual outputs retain model-scope metadata.

FMEA:

- Hybrid LP includes DC load shedding in EENS.
- DC customers included in SAIFI/SAIDI.
- Contingency details include both switching and repair stages.
- Repair search truncation surfaces in API.

F&D:

- Labeled as generation adequacy.
- Does not expose unavailable customer metrics as zero.

Three-stage:

- Per-component repair duration affects EENS once implemented.
- Branch-only catalog is reported in data/model scope.
- Hybrid fallback sets `ok = false` and exposes validity flags.

### GUI/API Tests

- Default run does not overwrite reliability data.
- User can select template explicitly.
- Result page displays model-scope badges.
- Result export contains validity, data quality, and limitations.
- Parameter table edits are reflected in the backend request.
- Method/model incompatible combinations are disabled or rejected with clear errors.

## Immediate High-Value Fixes

These are the best first debug tasks because they reduce misleading results quickly:

1. Change GUI defaults so reliability templates are opt-in.
2. Return model scope and validity from all reliability endpoints.
3. Preserve missing DC reliability/customer fields in JSON.
4. Fix hybrid SAIFI/SAIDI to include DC customers.
5. Create a shared reliability parameter resolver and migrate FMEA first.
6. Add tests for the resolver and DC customer metrics.

## Open Design Questions

1. Should MC remain AC adequacy only, or should it support the hybrid FMEA LP?
2. Should FMEA contingencies be rich assets, canonical elements, or both with provenance?
3. Should three-stage reliability become one of the four primary GUI methods or a separate restoration-reliability mode?
4. Should breaker and switch failures be asset outages, operation failures, or both?
5. Should missing reliability data block runs by default?
6. What is the official hours-per-year convention for this project: 8760 or 8736?
7. Should customer counts be stored on buses, loads, or both with a clear precedence rule?

## Proposed GUI Labels

Recommended method names:

- `Monte Carlo - non-sequential`
- `Monte Carlo - sequential`
- `FMEA / N-1 enumeration`
- `Frequency-duration generation adequacy`
- `Three-stage restoration reliability`

Recommended physical model names:

- `AC-only DC-OPF`
- `Hybrid AC/DC network LP`
- `AC LinDistFlow restoration MILP`
- `Generation adequacy COPT`
- `Full hybrid AC/DC OPF (future)`

Recommended scope badges:

- `AC only`
- `DC load excluded`
- `Hybrid load included`
- `DC fallback`
- `Voltage not certified`
- `Reactive feasibility not certified`
- `Generation adequacy only`
- `Branch-only contingencies`
- `Template data used`
- `Case data only`

## Conclusion

The reliability modules should be unified around a common data contract before deeper solver work. Once lambda, repair time, unavailability, component identity, customer weighting, and model-scope reporting are consistent, the four methods can remain different where they should be different: sampling versus enumeration, adequacy versus network reliability, and static OPF versus staged restoration.

The near-term goal should not be to force all methods to produce identical numbers. It should be to make them consume the same component data, report comparable metrics, and clearly explain which physical assumptions each result used.

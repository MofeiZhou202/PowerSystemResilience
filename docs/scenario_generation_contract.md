# Scenario Generation Runtime Contract

This document defines current behavior in `scenario_generation/`, whose public
headers live under `include/hacdcpf/analysis/`.

## Public entry points

`generate_regular_scenarios()`, `generate_reliability_scenarios()`, and
`generate_resilience_scenarios()` build one scenario family. `generate_scenarios()`
runs the enabled families and requires at least one enabled family.
`enumerate_n1_contingencies()` exposes the reliability contingency catalog.
JSON option/result conversion is provided by
`scenario_generation_options_from_json()` and
`scenario_generation_result_to_json()`.

Typhoon-specific entry points generate tracks, line segments, fault sequences,
catalogs, and road-network impact profiles. Catalog lookup returns
`shared_ptr<const TyphoonCatalog>` and sample lookup returns an aliasing
`shared_ptr<const TyphoonTrackSample>`; a caller-held sample therefore remains
valid while another thread refreshes a differently keyed catalog.

## Families and outputs

| Family | Candidate semantics | Additional authored reference |
|---|---|---|
| Regular | Load, renewable, storage-SOC, and climate perturbations over `TimeSeriesData`. | SSP/year and climate source are recorded in audit data. |
| Reliability | One group per enumerated FMEA-compatible N-1 contingency. | `component_index` is the stable source component `.index`; type/domain is carried separately. |
| Resilience | One group per requested typhoon category, using catalog tracks and generated failure/repair sequences. | Faults retain `ResilienceBranchKind` plus stable branch `.index`. |

Each `ScenarioCandidate` contains its probability, time series, features,
outage signature, risk scores, optional contingency/event, and generated
component profiles. Clustering returns a representative candidate, member IDs,
cluster probability, and distance audit. Candidate weights are normalized
within the generated group; cluster probabilities are normalized after
aggregation and are not unconditional probabilities across unrelated families.

## Randomness and reproducibility

Perturbation, climate, clustering, and typhoon options expose explicit seeds.
The same system, options, resource files, and implementation produce the same
pseudorandom sequence. Reproducibility does not extend across changes to input
ordering, catalog contents, resource files, or algorithm versions.

Perturbation multipliers are dimensionless. Time-series power profiles are MW;
storage state is MWh or per-unit SOC according to the destination field.
`num_steps` is a count and `time_step_hr` is hours.

## Typhoon and traffic units

Tracks use hours, decimal-degree latitude/longitude, hPa pressure deficit, km
radius, degrees heading, km/h translation speed, m/s wind, and degrees Celsius
SST. Segment length is km; rainfall is mm/h; fault and repair times are hours.
Road impact adds per-link, per-step wind, rainfall, surface water in mm,
dimensionless speed/capacity factors, and availability. It can write common
traffic free-flow-time, capacity, and availability profiles.

The typhoon wind field, rainfall relation, hydrologic bucket, fragility curves,
and road speed/capacity curves are engineering scenario models, not a weather
forecast or a hydraulic simulation. The road API validates all finite ranges
and ordered clamp bounds before computation.

## Fallback and approximation disclosure

- Missing climate CSV data can use an embedded SSP/year table or zero climate
  deltas. The choice is appended to `warnings` and family audit JSON.
- Missing component-level profile capacity can use aggregate profiles; audit
  data and warnings identify this fallback.
- Missing geographic data can use synthetic/affine coordinates. Typhoon and
  traffic results expose the corresponding flags.
- Large repair sets may use bounded approximate upstream-first ordering;
  `used_approximate_repair_order` and counts expose it.
- Resilience generation does not cross categories when the requested typhoon
  catalog group is short or empty; it uses available same-category samples or
  skips the group with a warning.
- Reliability inclusion flags do not extend coverage beyond component kinds
  implemented by the reliability FMEA catalog.

Consumers must preserve `warnings`, per-family `audit`, event fallback flags,
and `model_scope`; a representative scenario alone is not sufficient evidence
of full-tail coverage.

## Registered verification

- `test_scenario_generation`: climate morphing, deterministic perturbations,
  workload budgets, catalog coverage, and repair-order approximation.
- `test_typhoon_traffic_impact`: shared traffic profiles, coordinate fallback,
  immutable concurrent catalog snapshots, and invalid physical bounds.
- `test_scenario_bundle_schema`: exported multiplier dimensions and schema
  normalization.


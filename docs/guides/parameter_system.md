# Registered Parameter System

Updated: 2026-08-10

The editable engineering parameter registry is defined by
`StandardParameterLibrary` in
`include/hacdcpf/model/standard_parameter_library.hpp` and implemented in
`src/model/standard_parameter_library.cpp`.

## Contract

The supported value path is:

```text
compiled profile defaults
  -> external parameter-library JSON
  -> session API selection/update
  -> GUI editing
  -> explicit validation/apply
  -> effective_parameters echo
  -> numerical analysis
```

General importers remain source-faithful. Missing engineering data is not
silently filled through the generic registry. The IEC-CGE SVG adapter is an
explicit `BestEffort` exception because the source has no calculable electrical
asset record: it marks estimates by provenance and, by default, invokes the
same standards-aware design-handbook completion workflow. That behavior is
controllable through `auto_complete_parameters` and is returned in the import
report.

## Profiles and rule fields

The current profiles are `distribution_50hz` and
`lv_distribution_50hz`. Every `StandardParameterRule` declares:

- stable `id`, component type, and parameter name;
- editable default value and unit;
- optional lower and upper bounds;
- warning/error severity;
- source and engineering description.

The session response attaches read-only presentation metadata to every rule:
`symbol`, `quantity`, `model_role`, `equation`, `typical_min`, `typical_max`,
`typical_range_kind`, `typical_range_source`, and
`equivalent_circuit_family`. Typical ranges are engineering or nameplate
screening references. They are not API validation bounds, are never written
into the model, and do not replace manufacturer data, project design inputs,
test reports, or utility statistics. `min_value` / `max_value` remain the
active hard bounds used by backend validation. A null typical bound means that
the registry does not publish a responsible universal range.

The library itself is externally serializable. An update must include every
rule in the active profile; unknown IDs and invalid bounds are rejected.

## API and GUI

| Route | Behavior |
|---|---|
| `GET /api/session/parameter_library` | Returns profiles, rules, validation metadata, `effective_parameters`, `model_catalog`, and the current `parameter_instances` snapshot. |
| `POST /api/session/parameter_library/select` | Replaces the active profile by ID. |
| `POST /api/session/parameter_library/update` | Applies a complete external JSON override. |
| `POST /api/session/parameter_library/validate` | Diagnoses the current system without mutation. |
| `POST /api/session/parameter_library/apply` | Explicitly fills eligible missing/invalid values and reports changed fields. |

The backend-owned `model_catalog` currently enumerates 44 physical or system
model families serializable by `HybridPowerSystem`. The GUI selector is built
from this catalog rather than inferred from registry rules, so a model remains
selectable even when it has no independent standard-completion rule. The GUI
selects a component model and current instance, renders its solver scope and
equivalent circuit, and compares current instance, profile default, typical
screening, and hard validation values. The profile rows edit defaults, units,
hard bounds, severity, and source, import/export JSON, run validation, and
apply the selected profile. API responses expose
`effective_parameters` with value, origin, unit, source, editability, bounds,
and whether a rule was applied.

`parameter_instances` is a read-only snapshot of all fields emitted by the
authoritative `hacdcpf::io::to_json` system serialization. Nested objects and
arrays are flattened into field paths without substituting frontend defaults.
Each row uses the stable identity
`domain:component_kind:component .index`; vector position is not public.
Dedicated transformers and transformer-like AC branches retain distinct kinds,
and AC/DC storage identities are domain-qualified. Authored instance values
remain editable through the Canvas property editor; the model explorer edits
profile defaults and validation policy only.

The standard library still contains 52 underlying rules for compatibility and
JSON round trips. Reliability-prefixed rule groups are not exposed as peer
component models: the GUI overview currently shows the 24 physical standard
rules, while resolved failure modes and their parameter schema are attached to
each matching physical instance under the `Reliability` group. This uses
`reliability_kind + component .index` and the session's effective failure-mode
catalog. A component with no independent standard rule therefore still shows
its complete authored fields and resolved reliability parameters. Missing
typical or hard ranges are rendered as unpublished/unregistered; the GUI does
not invent engineering limits.

Implemented diagrams cover the AC line pi model, two-winding transformer T
equivalent, DC resistive branch, VSC AC impedance and conversion boundary,
DC/DC two-port, storage PCS/energy balance, AC/DC bus state, and system per-unit
base. The reliability section uses a state-transition block alongside the
selected physical model because failure and repair transitions are not
electrical equivalents.

The field-level distinction between standard tables, standard calculation
methods, and engineering assumptions is documented in
`distribution_parameter_completion_standards.md`.

## Numerical responsibility

Registry coverage does not mean every solver option is a model parameter.
Solver tolerances, iteration limits, parallel settings, dynamic integration
controls, harmonic options, and OPF algorithm settings remain request options
and must be echoed by their analysis result as `options_effective`,
`effective_parameters`, or an equivalent module-specific block.

For a parameter to be considered externally controllable, verify all six
stages:

1. default profile value;
2. external JSON override;
3. API acceptance;
4. GUI editability;
5. effective-value echo;
6. numerical sensitivity in a focused test.

The registry API E2E tests validate profile enumeration, invalid-parameter
diagnosis, explicit application, changed-field reporting, and effective bounds.

# Registered Parameter System

Updated: 2026-07-12

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

The library itself is externally serializable. An update must include every
rule in the active profile; unknown IDs and invalid bounds are rejected.

## API and GUI

| Route | Behavior |
|---|---|
| `GET /api/session/parameter_library` | Returns profiles, rules, validation metadata, and `effective_parameters`. |
| `POST /api/session/parameter_library/select` | Replaces the active profile by ID. |
| `POST /api/session/parameter_library/update` | Applies a complete external JSON override. |
| `POST /api/session/parameter_library/validate` | Diagnoses the current system without mutation. |
| `POST /api/session/parameter_library/apply` | Explicitly fills eligible missing/invalid values and reports changed fields. |

The GUI parameter table edits defaults and bounds, imports/exports JSON, runs
validation, and applies the selected profile. API responses expose
`effective_parameters` with value, origin, unit, source, editability, bounds,
and whether a rule was applied.

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

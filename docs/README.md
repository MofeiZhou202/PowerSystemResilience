# Documentation Index

Updated: 2026-08-09

This is the canonical entry point for active project documentation. Runtime
behavior is defined by `include/`, `src/`, `tests/run_gui_server.cpp`, `web/`,
and registered tests. Documentation describes that behavior; it does not
override it.

Historical audits, design proposals, and broad snapshots are isolated under
[`archive/`](archive/README.md). Archived material may explain prior decisions,
but it must not be used as evidence of current behavior.

## Document classes

| Class | Meaning |
|---|---|
| **Status** | Volatile build, test, dependency, and active-work evidence. |
| **Contract** | Current public behavior, inputs, outputs, limitations, and failure semantics. |
| **Implementation reference** | Source-backed derivation, manual, or verification material. Recheck line-level details after code changes. |
| **Archive** | Historical evidence or theory retained for provenance only. Not a current contract. |

The [module documentation map](module_documentation_map.md) evaluates coverage
across all major source modules and records missing focused contracts. The
[module code audit](module_code_audit.md) records review depth and open
source-backed defects.

## Start here

| Need | Class | Document |
|---|---|---|
| Current verified builds, dependency state, and active engineering work | Status | [Development status](development_status.md) |
| Cross-platform offline build and dependency profiles | Contract | [Cross-platform build](cross_platform_build.md) |
| Runtime HTTP routes and response boundaries | Contract | [Runtime API](runtime_api.md) |
| Built-in cases and capability-oriented examples | Contract | [Case catalog](case_catalog.md) |
| Python client and AI integration boundary | Contract | [Python API](python_api.md) |
| Module ownership and documentation gaps | Status | [Module documentation map](module_documentation_map.md) |
| Open code findings and module review depth | Status | [Module code audit](module_code_audit.md) |

## Core modeling and solvers

| Topic | Class | Primary document | Companion material |
|---|---|---|---|
| Component models and validation | Implementation reference | [Component-model audit source](ComponentModels/component_models_math_audit.tex) | [Registered parameter system](parameter_system.md) |
| Rich-to-canonical projection and result recovery | Contract | [Projection and result attribution](projection_and_results.md) | Component and power-flow manuals |
| Editable parameters and effective values | Contract | [Registered parameter system](parameter_system.md) | [Distribution parameter standards](distribution_parameter_completion_standards.md) |
| AC/DC and three-phase power flow | Implementation reference | [Power-flow manual source](PowerFlow/power_flow_manual1.tex) | [Harmonic verification](harmonics_analysis/harmonic_verification.md) |
| AC/DC, DC, parity, three-phase OPF, and RPO | Implementation reference | [OPF manual source](OptimalPowerFlow/opf_manual.tex) | Registered OPF and RPO tests |
| Network reconfiguration | Implementation reference | [Network reconfiguration models](network_reconfiguration_models.md) | [Certified restoration runtime](certified_restoration_runtime.md) |
| Hosting capacity | Implementation reference | [DL/T 2041-2025 implementation](capacity_analysis_implementation.md) | [Multidimensional weak-link identification](multidimensional_weak_link_identification.md) |

## Operations and planning

| Topic | Class | Primary document | Companion material |
|---|---|---|---|
| Time-series UC -> OPF -> PF | Implementation reference | [Time-series power-flow models](time_series_power_flow_models.md) | [Rich component models](sequential_production_simulation_rich_models.md) |
| Annual production and lifecycle | Implementation reference | [Annual simulation models](annual_simulation_models.md) | Time-series documents above |
| Day-ahead and real-time market | Contract | [Market runtime](market_simulation_runtime.md) | [Implemented mathematical model](market_simulation_mathematical_models.md) |
| Reliability | Implementation reference | [Consolidated reliability model](reliability_mathematical_models_and_intelligent_cyber_physical_assessment.md) | [Canonical reliability methods](reliability_assessment_models.md) |
| Resilience and restoration | Contract | [Certified restoration runtime](certified_restoration_runtime.md) | [Network reconfiguration models](network_reconfiguration_models.md) |
| EV and traffic inputs | Contract | [EV/traffic scenario format](ev_traffic_scenario_format.md) | Current headers and registered tests |

## Dynamics, faults, and power quality

| Topic | Class | Primary document | Companion material |
|---|---|---|---|
| Electromechanical transient runtime | Contract | [Transient runtime](transient_runtime.md) | Current dynamics headers and tests |
| Rich AC/DC short circuit | Implementation reference | [Short-circuit derivation and audit](short_circuit_rich_acdc_derivation.md) | Current headers under `include/hacdcpf/analysis/` |
| Harmonic power flow | Implementation reference | [Correctness verification](harmonics_analysis/harmonic_verification.md) | Current harmonic header and registered tests |

## Data, integration, and GUI

| Topic | Class | Document |
|---|---|---|
| IO governance and digital-twin readiness | Contract + labeled roadmap | [Digital-twin data/IO architecture](digital_twin_data_io_architecture.md) |
| CIM/CGMES 3.0 fields and round trip | Contract | [CIM/CGMES crosswalk](cim_cgmes3_crosswalk.md) |
| IEC-CGE distribution SVG import/export | Contract | [SVG distribution import](svg_distribution_import.md) |
| BPA/DSP cards and LCC integration | Implementation reference | [BPA/DSP component mapping](bpa_dsp_component_mapping.md) |
| Canvas rendering and result playback | Contract | [Canvas runtime](gui_canvas_runtime.md) |
| Commenting conventions | Contract | [Commenting guide](commenting_guide.md) |

## LaTeX manuals

The repository tracks editable `.tex` sources, not rendered PDFs or LaTeX
intermediates. Build and visually verify a PDF locally from the corresponding
directory when a rendered copy is needed:

- `ComponentModels/component_models_math_audit.tex`
- `PowerFlow/power_flow_manual1.tex`
- `OptimalPowerFlow/opf_manual.tex`

The old component-only PDF had no tracked editable source and was removed.

## Archive and local-only material

- [`archive/README.md`](archive/README.md) inventories retained theory,
  historical audits, and the legacy technical notebook. Archive links are
  intentionally absent from the active topic tables above.
- `docs/generated/` is ignored Doxygen output. Regenerate it from source; do
  not cite it as a canonical contract.
- `docs/latex/` is an ignored local research workspace. Files below it are not
  available in a clean clone and cannot satisfy a tracked documentation gap.
- LaTeX auxiliary files and rendered PDFs are build products and must remain
  untracked.

## Documentation policy

- Update a living contract instead of adding a dated audit snapshot.
- Keep volatile build and test evidence in `development_status.md`.
- Do not publish a roadmap, proposal, fallback, or archived audit as current
  behavior.
- Every public result vector must declare its index space and units.
- Every approximation must state its validity boundary and result flags.
- Runtime endpoint examples must use routes from `tests/run_gui_server.cpp`.
- Add new canonical documents here and update
  `module_documentation_map.md`; local, generated, or historical material stays
  outside the active index.

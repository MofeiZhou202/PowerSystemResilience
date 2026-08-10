# Module Documentation Map

Updated: 2026-08-09

This living map evaluates documentation coverage, not implementation quality.
Source, registered tests, and runtime responses remain authoritative. Update
this file in place when module ownership or canonical documents change; do not
create dated copies.

## Coverage labels

| Label | Meaning |
|---|---|
| **Focused** | A dedicated current contract or implementation manual exists. |
| **Distributed** | Useful active material exists, but the contract is spread across adjacent module documents. |
| **Theory-heavy** | Detailed derivation exists, but current runtime coverage is narrower or requires a separate contract. |
| **Uncovered** | No tracked, dedicated module document was found. Source headers and tests are the only reliable entry points. |

These labels do not mean that a solver is correct, complete, or production
ready. They describe how efficiently a maintainer can discover its current
contract and limitations.

## Focused contracts added

The formerly uncovered `graph/`, `scenario_generation/`, `carbon_analysis/`,
`integrated_energy/`, and `sppt/` modules now have tracked runtime contracts.
Their source-backed review and closure evidence remain in the
[module code audit](module_code_audit.md).

## Module coverage

| Source module | Coverage | Canonical documentation | Assessment |
|---|---|---|---|
| `model/`, `validation/` | Focused | [Component-model audit](ComponentModels/component_models_math_audit.tex), [parameter system](parameter_system.md) | Strong field/model coverage. Rendered PDFs are local build artifacts and are not tracked. |
| `projection/`, `assembly/` | Focused | [Projection and result attribution](projection_and_results.md), [power-flow manual](PowerFlow/power_flow_manual1.tex) | Contracts cover index spaces, recovery classes, SolverData, and matrix assembly. Preserve AC/DC domain-qualified maps in future examples. |
| `power_flow/` | Focused | [Power-flow manual](PowerFlow/power_flow_manual1.tex) | The broad manual remains active. The point-in-time mathematical audit is archived; open defects belong in the living code audit and tests. |
| `optimal_power_flow/` | Focused | [OPF manual](OptimalPowerFlow/opf_manual.tex) | Native, parity, DC, RPO, and three-phase paths are separated. Point-in-time IPM diagnostics and RPO validation are archived. |
| `power_models/` | Focused | OPF manual chapter `chapters/power_models.tex` | AML builders are documented inside the OPF manual; a separate document is unnecessary while that ownership stays clear. |
| `graph/` | Focused | [Graph and reduction runtime](graph_runtime_contract.md) | Stable/domain-qualified IDs, graph positions, reduction mappings, approximation boundaries, and recovery are explicit. |
| `network_reconfiguration/` | Focused | [Network reconfiguration models](network_reconfiguration_models.md), [certified restoration runtime](certified_restoration_runtime.md) | Canonical-space ONR and certified restoration are distinguished. |
| `reliability/` | Focused | [Consolidated reliability model](reliability_mathematical_models_and_intelligent_cyber_physical_assessment.md), [canonical methods](reliability_assessment_models.md) | Coverage is deep but duplicated. The consolidated document owns current status; focused documents should retain derivations without competing status claims. |
| `resilience/` | Distributed | [Certified restoration runtime](certified_restoration_runtime.md) | One restoration workflow is well specified; heuristic, multi-period MIP, staged MILP, and MESS routing lack a single module-wide result contract. |
| `analysis/` | Distributed | [Hosting capacity](capacity_analysis_implementation.md), [weak-link identification](multidimensional_weak_link_identification.md) | Major analyses have focused notes, but there is no umbrella contract for shared validity and attribution conventions. |
| `scenario_generation/` | Focused | [Scenario generation runtime](scenario_generation_contract.md) | Regular, reliability, resilience, typhoon, traffic-impact, reproducibility, and fallback contracts are explicit. |
| `short_circuit/` | Focused | [Short-circuit derivation and audit](short_circuit_rich_acdc_derivation.md) | AC and DC approximation boundaries are explicit; remember that public headers live under `include/hacdcpf/analysis/`. |
| `harmonics_power_flow/` | Distributed | [Correctness verification](harmonics_analysis/harmonic_verification.md) | The five theory notes are archived. A runtime contract should enumerate active algorithms, inputs, outputs, standards checks, and unsupported couplings. |
| `dynamics/` | Focused | [Transient runtime](transient_runtime.md) | The active runtime contract owns shipped behavior. Broader design and optional PSD interop material is archived. |
| `time_series/` | Focused | [Time-series PF](time_series_power_flow_models.md), [rich UC models](sequential_production_simulation_rich_models.md), [annual/lifecycle](annual_simulation_models.md) | Coverage is strong but overlapping. Each document now has a distinct owner: pipeline, component extensions, and annual/lifecycle orchestration. |
| `carbon_analysis/` | Focused | [Carbon analysis runtime](carbon_analysis_contract.md) | Snapshot/annual/GEC inputs, units, source IDs, AC/DC identity, and validity gates are explicit. |
| `ev_power_traffic/` | Distributed | [Scenario format](ev_traffic_scenario_format.md) | The active input schema is concise, while Formulations A-H and runtime result boundaries lack a focused implementation contract. The broad theory chapter is archived. |
| `integrated_energy/` | Focused | [Campus integrated energy runtime](integrated_energy_contract.md) | Carrier units, aggregate PCC scope, efficiencies, solver fallback, validity, and result mapping are explicit. |
| `market/` | Focused | [Market runtime](market_simulation_runtime.md), [mathematical model](market_simulation_mathematical_models.md) | Runtime and derivation are separated; AC-only and fallback boundaries must remain explicit. |
| `sppt/` | Focused | [SPPT executable runtime](sppt_runtime_contract.md) | MR observation semantics, certificates, guards, attribution, authority, benchmarks, and ablations are explicit. |
| `io/` | Focused | [Digital-twin architecture](digital_twin_data_io_architecture.md), [CIM/CGMES](cim_cgmes3_crosswalk.md), [SVG](svg_distribution_import.md), [BPA/DSP](bpa_dsp_component_mapping.md) | Broad format coverage exists. Contract/status/roadmap tags in the digital-twin document are essential because it mixes normative and proposed material. |
| `api/`, `src/server/`, `web/` | Focused | [Runtime API](runtime_api.md), [Python API](python_api.md), [Canvas runtime](gui_canvas_runtime.md), [case catalog](case_catalog.md) | User-visible routes and GUI contracts have focused entry points. Route examples must continue to match `tests/run_gui_server.cpp`. |

## Consolidation decisions

- `docs/README.md` owns navigation and document classification.
- `development_status.md` owns volatile build, test, dependency, and active
  investigation evidence.
- Focused contracts own current user-visible behavior. The legacy technical
  notebook and theory proposals are isolated under `docs/archive/` and must
  not be treated as active documentation.
- The consolidated reliability document owns current reliability status;
  smaller reliability documents retain focused derivations and rationale.
- The time-series documents own separate layers: pipeline, rich component
  formulations, and annual/lifecycle orchestration.
- Ignored `docs/generated/` and `docs/latex/` trees are not canonical sources
  and must not be the only target of a tracked link.

## Documentation priorities

This is a documentation backlog, not a product roadmap.

| Priority | Work | Completion criterion |
|---|---|---|
| P0 | Keep the canonical index clean-clone-safe. | Every local link from `docs/README.md` resolves to a tracked file. |
| P1 | Add a concise harmonic runtime contract. | Implemented algorithms and standards checks are separated from the five theory notes. |
| P1 | Complete module-wide result contracts for `resilience` and `ev_power_traffic`. | Every public result identifies scope, validity/fallback state, units, and authored/canonical index space. |
| P2 | Reduce status duplication in reliability and time-series documents. | Exactly one document owns current status for each workflow; companions link to it. |
| P2 | Define a reproducible build/refresh command for each LaTeX manual. | A maintainer can regenerate and visually verify each untracked PDF from tracked sources. |

## Update checklist

1. Inspect the relevant headers, implementations, registered tests, and CMake
   declarations before changing a coverage label.
2. Put current behavior in a focused contract and proposals in an explicitly
   marked theory section or theory document.
3. Declare result index spaces, units, fallback behavior, and unsupported
   coverage.
4. Add or change the primary link in `docs/README.md` and this map together.
5. Update `module_code_audit.md` when review depth or an open finding changes.
6. Run the repository link check, `git diff --check`, and a placeholder scan.

# Documentation Index

Updated: 2026-07-22

This directory separates implementation contracts from mathematical references.
When documentation and runtime behavior differ, `include/`, `src/`,
`tests/run_gui_server.cpp`, `web/`, and registered tests are authoritative.

## Start here

| Need | Document |
|---|---|
| Build on macOS, Linux, or Windows | [cross_platform_build.md](cross_platform_build.md) |
| Runtime HTTP endpoints, including monolithic three-phase hybrid PF/OPF | [runtime_api.md](runtime_api.md) |
| Built-in case catalog: capability-oriented showcase guide | [case_catalog.md](case_catalog.md) |
| Python SDK and AI enhancement architecture | [python_api.md](python_api.md) |
| Editable parameter registry and effective values | [parameter_system.md](parameter_system.md) |
| 中文 LaTeX：元件模型技术手册（41 类元件的参数、单位制、典型范围与等值电路） | [component_models/component_models.tex](component_models/component_models.tex) |
| Rich/canonical projection and result attribution | [projection_and_results.md](projection_and_results.md) |
| WebGL/SVG rendering, large-model interaction, and time playback | [gui_canvas_runtime.md](gui_canvas_runtime.md) |
| TSPF, annual simulation, storage, and profiles | [sequential_production_simulation_rich_models.md](sequential_production_simulation_rich_models.md) |
| Time-series PF pipeline: math models + analysis | [time_series_power_flow_models.md](time_series_power_flow_models.md) |
| Native hybrid AC/DC market: SCUC, DC storage, full-component N-1, settlement | [market_simulation_runtime.md](market_simulation_runtime.md) |
| 中文：当前市场模拟数学模型实现审核稿（含 DC 储能与全元件 N-1） | [market_simulation_mathematical_models.md](market_simulation_mathematical_models.md) |
| Annual simulation + lifecycle: math models + analysis | [annual_simulation_models.md](annual_simulation_models.md) |
| 中文 LaTeX：时序潮流/年度模拟数学模型 + 需求响应效应 | [latex/time_series_annual_simulation_zh.tex](latex/time_series_annual_simulation_zh.tex) |
| Transient runtime and validation boundary | [transient_runtime.md](transient_runtime.md) |
| Reliability methods | [reliability_assessment_models.md](reliability_assessment_models.md) |
| Cyber-physical reliability levels (L0–L4; Level 1 implemented) | [cyber_physical_reliability_extension.md](cyber_physical_reliability_extension.md) |
| Consolidated reliability mathematics + intelligent cyber-physical extension | [reliability_mathematical_models_and_intelligent_cyber_physical_assessment.md](reliability_mathematical_models_and_intelligent_cyber_physical_assessment.md) |
| IEEE Transactions on Reliability draft: progressive class-conditioned frequency--duration method | [latex/Progressive Class-Conditioned Frequency–Duration/main.tex](latex/Progressive%20Class-Conditioned%20Frequency%E2%80%93Duration/main.tex) |
| Short-circuit methods | [short_circuit_rich_acdc_derivation.md](short_circuit_rich_acdc_derivation.md) |
| Network reconfiguration | [network_reconfiguration_models.md](network_reconfiguration_models.md) |
| Certified restoration master-oracle runtime | [certified_restoration_runtime.md](certified_restoration_runtime.md) |
| RPO structure (OLTC + continuous) and cross-validation | [reactive_power_optimization_validation.md](reactive_power_optimization_validation.md) |
| Large hybrid OPF IPM diagnostics | [large_hybrid_ipm_diagnostics.md](large_hybrid_ipm_diagnostics.md) |
| Multidimensional weak-link identification | [multidimensional_weak_link_identification.md](multidimensional_weak_link_identification.md) |
| Hosting capacity (DL/T 2041-2025) implementation | [../capacity_analysis_implementation.md](../capacity_analysis_implementation.md) |
| CIM/CGMES 3.0 field crosswalk and round-trip contract | [cim_cgmes3_crosswalk.md](cim_cgmes3_crosswalk.md) |
| PowerSimulationsDynamics.jl interop validation | [powersimulationsdynamics_interop.md](powersimulationsdynamics_interop.md) |
| Digital-twin and IO architecture | [digital_twin_data_io_architecture.md](digital_twin_data_io_architecture.md) |
| EV/traffic scenario schema | [ev_traffic_scenario_format.md](ev_traffic_scenario_format.md) |
| Code commenting conventions | [commenting_guide.md](commenting_guide.md) |
| BPA/DSP 元件卡片对照、LCCConverter 集成与四案例潮流对比 | [bpa_dsp_component_mapping.md](bpa_dsp_component_mapping.md) |

## Technical notebook

The implementation-facing LaTeX notebook starts at
[`technical_notebook/main.tex`](technical_notebook/main.tex). It contains the
data model, solver formulations, IO dictionaries, projection contracts,
analysis boundaries, and algorithm index. Generated `main.pdf` is a build
artifact; the `.tex` sources are authoritative.

## Theory references

The following documents contain derivations or research-oriented design
material. They are retained because the equations are useful, but they are not
claims that every described model is enabled in production:

- `Updated_GFL_GFM_Control_Modeling.md`
- `dynamics_electromechanical_transient_design.md`
- `multiple_converter.md` and `multiple_converter_r1.md`
- `harmonics_analysis/harmonic_*.md`
- `technical_notebook/sections/fleet_ev_icv_ctm_ltm_theoretical_model.md`

Each theory document must be interpreted together with the relevant runtime
contract and tests.

## Documentation policy

- Do not add dated audit snapshots when an implementation contract can be
  updated instead.
- Do not publish a roadmap as current behavior.
- Every public result vector must declare its index space and units.
- Every approximation must name its validity boundary.
- Runtime endpoint examples must use production routes from
  `tests/run_gui_server.cpp`.
- Temporary generated Markdown belongs outside the canonical index and may be
  deleted after its source data is preserved.

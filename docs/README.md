# Documentation Index

Updated: 2026-07-12

This directory separates implementation contracts from mathematical references.
When documentation and runtime behavior differ, `include/`, `src/`,
`tests/run_gui_server.cpp`, `web/`, and registered tests are authoritative.

## Start here

| Need | Document |
|---|---|
| Build on macOS, Linux, or Windows | [cross_platform_build.md](cross_platform_build.md) |
| Runtime HTTP endpoints | [runtime_api.md](runtime_api.md) |
| Editable parameter registry and effective values | [parameter_system.md](parameter_system.md) |
| Rich/canonical projection and result attribution | [projection_and_results.md](projection_and_results.md) |
| Canvas result rendering and time playback | [gui_canvas_runtime.md](gui_canvas_runtime.md) |
| TSPF, annual simulation, storage, and profiles | [sequential_production_simulation_rich_models.md](sequential_production_simulation_rich_models.md) |
| Time-series PF pipeline: math models + analysis | [time_series_power_flow_models.md](time_series_power_flow_models.md) |
| Annual simulation + lifecycle: math models + analysis | [annual_simulation_models.md](annual_simulation_models.md) |
| Transient runtime and validation boundary | [transient_runtime.md](transient_runtime.md) |
| Reliability methods | [reliability_assessment_models.md](reliability_assessment_models.md) |
| Short-circuit methods | [short_circuit_rich_acdc_derivation.md](short_circuit_rich_acdc_derivation.md) |
| Network reconfiguration | [network_reconfiguration_models.md](network_reconfiguration_models.md) |
| Digital-twin and IO architecture | [digital_twin_data_io_architecture.md](digital_twin_data_io_architecture.md) |
| EV/traffic scenario schema | [ev_traffic_scenario_format.md](ev_traffic_scenario_format.md) |

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


# Hybrid AC/DC C++ Notebook {#mainpage}

This generated notebook is meant for code review: data structures, public APIs,
implementation files, and the relations between functions/classes.

## Start Here

- [Main project overview](README.md)
- [Class index](annotated.html)
- [File index](files.html)
- [Namespace index](namespaces.html)
- [Data structures](classes.html)
- [Member list](functions.html)
- [Commenting guide](@ref commenting_guide)

## Core Review Path

Read the project in this order when you want the shortest path from engineering
data to numerical results.

1. Data model: `include/hacdcpf/model/hybrid_power_system.hpp`,
   `include/hacdcpf/model/ac_components.hpp`,
   `include/hacdcpf/model/dc_components.hpp`, and
   `include/hacdcpf/model/converter_components.hpp`.
2. Validation and projection:
   `include/hacdcpf/validation/validate_system.hpp`,
   `include/hacdcpf/projection/project_to_canonical.hpp`, and
   `src/model/network_utils.cpp`.
3. Converter control and model-scope declarations:
   `include/hacdcpf/model/device_control_role.hpp`,
   `include/hacdcpf/model/converter_model_scope.hpp`,
   `include/hacdcpf/power_flow/converter_coordination.hpp`, and
   `src/power_flow/converter_coordination.cpp`.
4. Solver assembly:
   `include/hacdcpf/assembly/solver_data.hpp`,
   `include/hacdcpf/assembly/index_map.hpp`, and
   `src/power_flow/solver_data.cpp`.
5. Power flow:
   `include/hacdcpf/power_flow/*.hpp` and `src/power_flow/*.cpp`.
6. Optimal power flow:
   `include/hacdcpf/optimal_power_flow/*.hpp` and
   `src/optimal_power_flow/*.cpp`.
7. Graph and topology tools:
   `include/hacdcpf/graph/*.hpp` and `src/graph/*.cpp`.
8. I/O and application API:
   `include/hacdcpf/io/*.hpp`, `src/io/*.cpp`,
   `include/hacdcpf/api/hacdcpf.hpp`, and `src/api/hacdcpf.cpp`.
9. IO governance and digital-twin readiness:
   `include/hacdcpf/io/component_io_mapping.hpp`,
   `src/io/component_io_mapping.cpp`, and
   [IO digital-twin review](@ref io_digital_twin_review).

## Useful Notebook Queries

Use the search box in the generated HTML for these review terms:

- `HybridPowerSystem`
- `SolverData`
- `PowerFlowResult`
- `SolverDiagnostics`
- `ConverterCoordinationReport`
- `ConverterModelScope`
- `DeviceControlRole`
- `OPFProblem`
- `ACOPFResult`
- `DCOPFResult`
- `Formulation`
- `project_to_canonical_models`
- `make_solver_data`
- `solve_power_flow`
- `solve_ac_opf`
- `build_power_system_graph`
- `component_io_mappings`
- `analyze_component_parameter_quality`
- `analyze_digital_twin_readiness`
- `DigitalTwinReadinessReport`

## Current Result Metadata

The current public result objects deliberately expose model fidelity rather than
requiring callers to infer it from the solver name.

- `PowerFlowResult::converter_model_scope` reports the converter physics honored
  by the snapshot Newton solve. `PowerFlowResult::diagnostics` also contains
  `converter_coordination`, equation-closure scan fields, `promoted_vsc_indices`,
  and `effective_converters`.
- `DCDCTransfer` reports the solved duty ratio, voltage ratio, and whether the
  duty ratio is defined/feasible for the declared topology.
- `ACOPFResult` carries `solver_path`, `profiling`, `infeasibility_hints`,
  `audit`, and `converter_model_scope`. The parity path reflects enabled
  converter capacity/current/modulation/DC-DC-duty constraint families in the
  scope tag and validity flags.
- `DCOPFResult` carries `solver_chain`, `objective_model`,
  `load_shedding_mw`, `branch_mu_valid`, and `converter_model_scope`.

## IO Governance and Digital-Twin Readiness

The `数据IO` module now exposes machine-readable governance data rather than
only import/export handlers.

- `component_io_mappings()` declares representation policy for native JSON,
  canonical projection, GridLAB-D, and OpenDSS.
- `component_parameter_rules()` exposes range/presence rules for static,
  dynamic, transient, failure, and reliability parameters.
- `analyze_component_parameter_quality()` applies those rules to a
  `HybridPowerSystem`.
- `analyze_digital_twin_readiness()` returns an L0--L5 maturity report with
  eleven criteria spanning identity, topology, parameters, dynamics, telemetry,
  state synchronization, events, reliability, standards interoperability,
  numerical validation, and provenance.

For a focused review, start at [IO digital-twin review](@ref
io_digital_twin_review).

## Adding Better Function Notes

Doxygen will list every symbol even when comments are missing. Add comments to
important interfaces when you want the notebook to become more explanatory:

```cpp
/// Solves an AC optimal power flow problem for a projected system.
/// @param system Rich or canonicalized system data.
/// @param options Solver tolerances, limits, and backend choices.
/// @return Dispatch, objective value, dual/status data, and diagnostics.
OPFResult solve_ac_opf(const HybridPowerSystem& system,
                       const OPFOptions& options);
```

Keep comments on public headers concise. Put longer algorithm notes in Markdown
files under `docs/` and link to the related headers/functions.

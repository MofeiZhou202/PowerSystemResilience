# Hybrid AC/DC C++ Notebook {#mainpage}

This generated notebook is meant for code review: data structures, public APIs,
implementation files, and the relations between functions/classes.

## Start Here

- [Main project overview](md_README.html)
- [Class index](annotated.html)
- [File index](files.html)
- [Namespace index](namespaces.html)
- [Data structures](classes.html)
- [Member list](functions.html)

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
3. Solver assembly:
   `include/hacdcpf/assembly/solver_data.hpp`,
   `include/hacdcpf/assembly/index_map.hpp`, and
   `src/power_flow/solver_data.cpp`.
4. Power flow:
   `include/hacdcpf/power_flow/*.hpp` and `src/power_flow/*.cpp`.
5. Optimal power flow:
   `include/hacdcpf/optimal_power_flow/*.hpp` and
   `src/optimal_power_flow/*.cpp`.
6. Graph and topology tools:
   `include/hacdcpf/graph/*.hpp` and `src/graph/*.cpp`.
7. I/O and application API:
   `include/hacdcpf/io/*.hpp`, `src/io/*.cpp`,
   `include/hacdcpf/api/hacdcpf.hpp`, and `src/api/hacdcpf.cpp`.

## Useful Notebook Queries

Use the search box in the generated HTML for these review terms:

- `HybridPowerSystem`
- `SolverData`
- `PowerFlowResult`
- `OPFProblem`
- `Formulation`
- `project_to_canonical_models`
- `make_solver_data`
- `solve_power_flow`
- `solve_ac_opf`
- `build_power_system_graph`

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

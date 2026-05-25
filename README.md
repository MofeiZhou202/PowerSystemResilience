# HybridACDCDistributionSystemsSimulation

A C++20 static library (`hacdcpf`) for steady-state simulation, optimisation, and analysis of hybrid AC/DC distribution power systems.

---

## Overview

The library covers power flow, optimal power flow, reliability, resilience, carbon analysis, and time-series production simulation for networks that mix AC feeders, DC buses, VSC converters, Soft Open Points (SOPs), microgrids, battery storage, PV systems, and mobile storage units.

The three-stage MILP fault-recovery reliability evaluator is implemented natively in C++ using the HiGHS/Ipopt backends provided by MIPSolvers.

---

## Repository Layout

```
.
├── CMakeLists.txt                 # Top-level build
├── cmake/
│   └── Dependencies.cmake         # MIPSolvers sibling-dir + Catch2 resolution
├── include/hacdcpf/
│   ├── api/                       # Public façade (hacdcpf.hpp)
│   ├── model/                     # HybridPowerSystem, AC/DC/converter components
│   ├── power_flow/                # PF options, result types, solver interfaces
│   ├── optimal_power_flow/        # OPF options, result types
│   ├── power_models/              # Algebraic model builders (LinDistFlow, SCUC…)
│   ├── analysis/                  # Short-circuit, three-stage reliability headers
│   ├── carbon_analysis/           # Carbon tracing interfaces
│   ├── time_series/               # UC / time-series / lifecycle interfaces
│   ├── reliability/               # Monte Carlo reliability, FMEA interfaces
│   ├── resilience/                # Distribution resilience interfaces
│   ├── io/                        # JSON, MATPOWER, optional Excel/OpenDSS
│   ├── graph/                     # Graph analysis and reduction
│   └── engine/                    # Forwarding headers into mipsolvers::engine
├── src/                           # Implementation files (mirrors include tree)
├── tests/                         # Catch2 test executables
├── data/                          # MATPOWER .m case files
└── docs/technical_notebook/       # LaTeX implementation handbook (main.pdf)
```

---

## Dependencies

| Dependency | How obtained | Notes |
|---|---|---|
| **MIPSolvers** | Sibling directory `../MIPSolvers` | Provides Eigen 3, `{fmt}`, `nlohmann/json`, HiGHS LP/QP, and optional Ipopt. **Required.** |
| **Catch2** v3.5.4 | `FetchContent` (or reused from sibling build dirs) | Test-only; not linked into the library. |
| **OpenXLSX** | `HACDCPF_ENABLE_EXCEL=ON` | Optional; needed only for Excel workbook I/O. |
| **dss_capi** | `HACDCPF_ENABLE_OPENDSS=ON` | Optional; needed only for the OpenDSS bridge. |

No Homebrew or system-installed solver library is searched.

---

## Building

```bash
# 1. Clone MIPSolvers alongside this repo
git clone <mipsolvers-url> ../MIPSolvers

# 2. Configure (Debug build shown; use build_rel/ for Release)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug

# 3. Build
cmake --build build -j$(nproc)

# 4. Run tests
ctest --test-dir build --output-on-failure
```

### CMake Options

| Option | Default | Effect |
|---|---|---|
| `HACDCPF_BUILD_TESTS` | `ON` | Build Catch2 test executables |
| `HACDCPF_ENABLE_EXCEL` | `OFF` | Include Excel workbook I/O (`src/io/excel_io.cpp`) |
| `HACDCPF_ENABLE_OPENDSS` | `OFF` | Include OpenDSS bridge (`src/io/opendss_bridge.cpp`) |
| `HACDCPF_ENABLE_WERROR` | `OFF` | Treat compiler warnings as errors |
| `HACDCPF_ENABLE_NATIVE_ARCH` | `OFF` | Enable `-march=native` |

---

## Public API

Include the façade header and link against `hacdcpf::hacdcpf`:

```cpp
#include "hacdcpf/api/hacdcpf.hpp"
```

### Power Flow

```cpp
// Hybrid AC/DC Newton-Raphson (default)
PowerFlowResult r = hacdcpf::solve_power_flow(sys);

// Specialised variants
DCPowerFlowResult  dc  = hacdcpf::solve_dc_power_flow(sys);
AdaptiveSolveResult ad  = hacdcpf::solve_power_flow_adaptive(sys);
IslandedSolveResult isl = hacdcpf::solve_power_flow_islanded(sys);

// Fast-decoupled PF
PowerFlowResult fdpf = hacdcpf::solve_power_flow_fdpf(sys);

// AC/DC linearised (for quick approximations)
powerflow::ACLinearizedDCResult lin = hacdcpf::solve_ac_dc_power_flow(sys);

// Persistent handle for repeated solves on the same topology
SolverHandle* h = hacdcpf::create_solver_handle(sys);
PowerFlowResult r2 = hacdcpf::solve_handle(h);
hacdcpf::destroy_solver_handle(h);

// Three-phase Newton-Raphson (ThreePhaseACSystem)
ThreePhaseDPFResult tpf = hacdcpf::analysis::solve_three_phase_nr(tpsys);
```

### Optimal Power Flow

```cpp
opf::ACOPFResult ac = hacdcpf::solve_ac_opf(sys);   // native IPM / parity-IPM
opf::DCOPFResult dc = hacdcpf::solve_dc_opf(sys);   // LP / QP backends
opf::RPOResult   rp = hacdcpf::solve_rpo(sys);       // reactive-power optimisation
```

### Time-Series and Production

```cpp
TimeSeriesPFResult ts = hacdcpf::solve_time_series_pf(sys, scenarios, opt);
UCSchedule         uc = hacdcpf::solve_unit_commitment(sys, data, opt);
AnnualProductionResult ap = hacdcpf::solve_annual_production_simulation(sys, opt);
LifecycleResult    lc = hacdcpf::run_lifecycle_simulation(sys, opt);
```

### Reliability and Resilience

```cpp
// Monte Carlo reliability (non-sequential or sequential)
ReliabilityResult mc = analysis::run_nonsequential_mc(sys);
ReliabilityResult sq = analysis::run_sequential_mc(sys, load_profile);

// FMEA (N-1 failure-mode enumeration)
FMEAResult fmea = analysis::run_distribution_fmea(sys);

// Multi-hour distribution resilience (heuristic sequential)
DistributionResilienceResult res =
    hacdcpf::run_distribution_resilience_assessment(sys);

// Three-stage MILP reliability (native C++ MILP via MIPSolvers)
ThreeStageReliabilityResult r3s =
    analysis::run_three_stage_reliability(case_json_path);
```

### Analysis

```cpp
// Short-circuit (IEC 60909 / simplified sequence network)
ShortCircuitResult sc = analysis::run_short_circuit_analysis(sys);

// Carbon tracing (BFS proportional + matrix method)
CarbonAnalysisResult co2 = hacdcpf::run_carbon_analysis(sys, pf_result);

// Network validation
ValidationReport vr = hacdcpf::validate_full(sys);
```

---

## Data Model

The central type is `HybridPowerSystem`, which aggregates:

- **`ACSystem`** — buses, branches, generators, loads, transformers (2W/3W), switches, circuit breakers, static generators, renewables, storage, PV systems, EV chargers, flexible loads, asymmetric loads, external grids, shunts, wardrobes, VPPs, microgrids
- **`DCSystem`** — DC buses, DC branches, DC generators, DC loads, DC/DC converters
- **`std::vector<VSCConverter>`** — VSC coupling links between AC and DC buses
- **`ThreePhaseACSystem`** (optional) — explicit abc-domain three-phase buses, lines, transformers, loads, generators, regulators
- **Mobile storage, charging stations, energy routers**

A projection layer (`project_to_canonical_models`) converts the rich component set to solver-ready canonical models before numerical assembly.

---

## I/O

| Format | Read | Write | Notes |
|---|---|---|---|
| MATPOWER `.m` | ✓ | — | `load_matpower_case` |
| JPC JSON | ✓ | ✓ | `load_jpc_json` / `save_jpc_json` |
| PF result JSON | — | ✓ | `save_power_flow_result_json` |
| OPF result JSON | ✓ | ✓ | `opf_result_to_json` / `opf_result_from_json` |
| Carbon result JSON | ✓ | ✓ | `carbon_result_to_json` / `carbon_result_from_json` |
| Excel workbook | ✓ | ✓ | Requires `HACDCPF_ENABLE_EXCEL=ON` |
| OpenDSS | ✓ | — | Requires `HACDCPF_ENABLE_OPENDSS=ON` |

---

## Test Suite

| Executable | Coverage area |
|---|---|
| `test_hacdcpf` | High-level API smoke tests, resilience stub |
| `test_all_components_pf` | Rich-component projection and power flow |
| `test_acopf_dcopf_crossval` | AC/DC OPF cross-validation |
| `test_matpower_cases` | Tier-1/2/3 MATPOWER convergence regression (100+ cases) |
| `test_three_phase_pf` | Three-phase NR, transformers, distribution PF boundary |
| `test_three_phase_nr` | Three-phase NR edge cases |
| `test_resilience_assessment` | Resilience heuristic model, MESS dispatch |
| `test_three_stage_reliability` | Three-stage bridge and JSON parsing |
| `test_short_circuit_crossval` | Short-circuit numerical cross-validation |
| `test_io_json` | JSON I/O round-trips |
| `test_rich_components` | Transformer 2W/3W, PV, storage, VSC |
| `test_validation` | `ValidationReport` and deliberate-fault injection |
| `test_graph`, `test_graph_kron` | Graph reduction and Kron elimination |
| `test_topology_crossval` | Topology analysis cross-validation |
| `test_advanced_pf` | Multi-island, distributed-slack, PV/PQ switching |
| `test_multiport_vpp_er` | Multi-port EnergyRouter, VPP control modes |
| `test_scaling_regression` | Numerical scaling regression |
| `test_powerflow_performance` | Performance benchmarks (release flags) |

Run all tests:

```bash
ctest --test-dir build --output-on-failure
```

---

## Known Limitations

The following items are present in the public API but have documented implementation gaps. See `docs/technical_notebook/sections/18_implementation_gap_audit.tex` and the compiled handbook (`docs/technical_notebook/main.pdf`) for the full audit.

| Area | Status |
|---|---|
| AC OPF fallback path | Fast economic-dispatch plus AC-PF fallback can return `converged` for AC-only recovery; hybrid DC/converter subsystems ignored by that path are noted in the status string. |

---

## Documentation

The implementation-facing technical handbook lives in `docs/technical_notebook/`. Build it with:

```bash
cd docs/technical_notebook
pdflatex -interaction=nonstopmode main.tex
pdflatex -interaction=nonstopmode main.tex   # second pass for cross-references
```

A pre-built `main.pdf` is committed to the repository.

---

## License

See [LICENSE](LICENSE).

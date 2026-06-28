# Cross-Platform Build Configuration

This project consumes `MIPSolvers` as source, usually from the sibling checkout
`../MIPSolvers`.  The project CMake configures the solver dependency with a
portable default profile so the same checkout can build on macOS, Linux, and
Windows without editing vendored solver sources.

## Dependency Layout

Recommended layout:

```text
Codes/
  HybridACDCDistributionSystemsSimulation/
  MIPSolvers/
```

If `MIPSolvers` is somewhere else, pass:

```bash
cmake -S . -B build -DMIPSOLVERS_SOURCE_DIR=/absolute/path/to/MIPSolvers
```

## Presets

From `HybridACDCDistributionSystemsSimulation`:

```bash
cmake --preset macos-release
cmake --build --preset macos-release
ctest --preset macos-release
```

```bash
cmake --preset linux-release
cmake --build --preset linux-release
ctest --preset linux-release
```

```powershell
cmake --preset windows-vcpkg-release
cmake --build --preset windows-vcpkg-release
ctest --preset windows-vcpkg-release
```

Use `portable-no-suitesparse` when you want the most conservative dependency
surface and accept Eigen's built-in sparse LU fallback:

```bash
cmake --preset portable-no-suitesparse
cmake --build --preset portable-no-suitesparse
```

## Platform Prerequisites

### macOS

```bash
brew install cmake ninja fmt nlohmann-json suite-sparse
```

The `macos-release` preset enables embedded Ipopt through `MIPSolvers`, matching
the currently supported Ipopt path in that checkout.

### Linux

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake ninja-build \
  libfmt-dev nlohmann-json3-dev libeigen3-dev \
  libsuitesparse-dev libsuperlu-dev
```

Embedded Ipopt is OFF by default on Linux because the current
`MIPSolvers/cmake/BuildIpopt.cmake` path is macOS-specific.

### Windows

Use an x64 Native Tools Prompt or a Developer PowerShell for Visual Studio 2022.

```powershell
git clone https://github.com/microsoft/vcpkg C:\vcpkg
C:\vcpkg\bootstrap-vcpkg.bat
C:\vcpkg\vcpkg install fmt:x64-windows nlohmann-json:x64-windows eigen3:x64-windows suitesparse:x64-windows superlu:x64-windows

$env:VCPKG_ROOT = "C:\vcpkg"
$env:SUITESPARSE_ROOT = "C:\vcpkg\installed\x64-windows"
cmake --preset windows-vcpkg-release
cmake --build --preset windows-vcpkg-release
```

The Windows preset uses the dynamic MSVC runtime (`/MD`) so vcpkg
`x64-windows` packages link consistently.  If you intentionally use
`x64-windows-static`, configure manually with the matching triplet and
`CMAKE_MSVC_RUNTIME_LIBRARY`.

## OPF Linear Solver Runtime Switch

The parity OPF IPM supports:

```text
HACDCPF_OPF_LINEAR_SOLVER=auto|dense|umfpack|klu|eigen
```

For Windows deployment:

- `auto` uses dense LU for small KKT systems on Windows and sparse solvers for
  larger systems when available.
- `umfpack` is the preferred explicit sparse choice when SuiteSparse is
  installed and linked correctly.
- `klu` is the second sparse choice.
- `eigen` forces Eigen `SparseLU` and needs no external sparse library.

PowerShell examples:

```powershell
$env:HACDCPF_OPF_LINEAR_SOLVER = "umfpack"
.\build\windows-vcpkg-release\Release\opf_numerical_benchmark.exe data case9.m case30.m
```

```powershell
$env:HACDCPF_OPF_LINEAR_SOLVER = "dense"
.\build\windows-vcpkg-release\Release\opf_numerical_benchmark.exe data case9.m case30.m
```

Check the benchmark output field `linear_solver_backend`.  If a forced
`umfpack` run reports `sparse_eigen_lu`, SuiteSparse was not discovered by CMake
or the executable was not linked to the SuiteSparse libraries.

## CMake Options Owned by This Project

| Option | Default | Meaning |
|---|---:|---|
| `HACDCPF_DEPENDENCY_PROFILE` | `portable` | Controls how much of `MIPSolvers` is built: `portable`, `full`, or `minimal`. |
| `HACDCPF_USE_SUITESPARSE` | `ON` | Enables SuiteSparse discovery for both `MIPSolvers` and the OPF KKT backend. |
| `HACDCPF_USE_GUROBI` | `OFF` | Keeps Gurobi disabled even when it is installed locally; set `ON` only for deliberate Gurobi validation. |
| `HACDCPF_SUITESPARSE_ROOT` | empty | SuiteSparse prefix; also exported as `SUITESPARSE_ROOT` for `MIPSolvers`. |
| `HACDCPF_ENABLE_IPOPT` | `ON` on macOS, `OFF` elsewhere | Builds embedded Ipopt only where the current `MIPSolvers` CMake supports it. |
| `HACDCPF_ENABLE_NATIVE_ARCH` | `OFF` | Propagates host-specific CPU tuning to this project and `MIPSolvers`. |

The `portable` profile builds embedded HiGHS/SCIP/HFactor but disables
MIPSolvers tests, SCUC command-line tools, and Python bindings while used as a
subproject.  Use `full-dev` when actively developing the solver dependency too.

Gurobi is intentionally disabled by default on macOS, Linux, and Windows.  The
Auto solver chains should therefore use native/HiGHS/SCIP paths and report
`has_gurobi=false` from `get_solver_capabilities()`.

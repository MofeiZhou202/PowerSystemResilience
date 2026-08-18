> Documentation Sync (2026-07-26)
> Scope: offline source builds on macOS, Linux, and Windows.
> Status: implementation-backed reference.
> Source of truth: `CMakeLists.txt`, `CMakePresets.json`, and
> `cmake/Dependencies.cmake` override this document if they diverge.

# Cross-Platform Offline Build

The normal build does not download dependencies. `MIPSolvers` supplies complete
local source copies of Eigen 3.4.1, fmt, nlohmann/json, HiGHS, SCIP, MUMPS,
Ipopt, SuiteSparse, Catch2, PaPILO 3.0.0, and PaPILO's required Boost headers.
This repository supplies OpenXLSX for ETAP Excel I/O. Eigen includes both the
core tree and `unsupported/`, including
`unsupported/Eigen/MatrixFunctions`.

## Source Package Layout

The default layout is:

```text
package/
  HybridACDCDistributionSystemsSimulation/
  MIPSolvers/
```

The `MIPSolvers` directory may instead be placed inside the main repository or
selected explicitly:

```bash
cmake -S . -B build/release \
  -DMIPSOLVERS_SOURCE_DIR=/absolute/path/to/MIPSolvers
```

Release and CI builds reject a dirty Git checkout of MIPSolvers. A distributed
source archive without `.git` metadata is accepted and reported as such; the
archive producer is responsible for preserving the pinned contents.

Do not omit these paths from an offline package:

```text
MIPSolvers/third_party/eigen/
MIPSolvers/third_party/fmt/
MIPSolvers/third_party/nlohmann_json/
MIPSolvers/third_party/catch2/
MIPSolvers/third_party/papilo/
MIPSolvers/third_party/boost_papilo/
MIPSolvers/highs/  MIPSolvers/scip/  MIPSolvers/mumps/
MIPSolvers/ipopt/  MIPSolvers/suitesparse/
MIPSolvers/third_party/install/
HybridACDCDistributionSystemsSimulation/third_party/OpenXLSX-master/
```

On Windows, `MIPSolvers/third_party/install` is produced from a staged static
oneMKL bundle and is ABI-specific. Keep it together with its manifest and
oneMKL license notices; do not reuse it across MSVC toolsets, architectures,
runtime-library modes, or Release/Debug configurations.

## Toolchain Prerequisites

Only build tools and platform runtimes are prerequisites. CMake 3.20 or newer
and a C++20 compiler are required on every platform.

### macOS

Use Xcode Command Line Tools. The default embedded Ipopt/MUMPS profile also
needs a Fortran compiler such as `gfortran`. Homebrew libraries are not build
dependencies; Homebrew may still be used to install CMake or the compiler.

```bash
cmake --preset macos-release
cmake --build --preset macos-release
ctest --preset macos-release
```

### Linux

Use GCC or Clang with the standard C/C++ build tools. The main Linux preset
keeps embedded Ipopt off, so a Fortran compiler is not required for the default
profile.

```bash
cmake --preset linux-release
cmake --build --preset linux-release
ctest --preset linux-release
```

### Windows

Use an x64 Native Tools Prompt or Developer PowerShell for Visual Studio 2022.
Prepare MIPSolvers once on a connected Windows staging machine:

```powershell
cd ..\MIPSolvers
.\third_party\stage_onemkl.ps1 -SourceRoot $env:MKLROOT -Force
.\third_party\build_third_party.ps1 -Jobs 8 -BuildType Release -Fresh
cd ..\HybridACDCDistributionSystemsSimulation
```

Transfer both repositories, including the ignored
`MIPSolvers/third_party/install` artifact, into the sealed environment. The
staging script records SHA-256 hashes and license material; the prebuilt
package exports Ipopt plus a relocatable static `MIPSolvers::MKL` target. Its
post-install consumer check is configure-only, so it validates target scope and
paths without another compile or link pass. `build_third_party.ps1` and the
main Windows build preset cap parallelism at 8 and keep IPO disabled.

```powershell
cmake --preset windows-msvc-release
cmake --build --preset windows-msvc-release
ctest --preset windows-msvc-release
```

The preset uses the dynamic MSVC runtime (`/MD` in Release), requires the
compatible MIPSolvers prebuilt package, and enables embedded Ipopt with the
sequential static PardisoMKL backend. `windows-vcpkg-release` remains a
compatibility profile for sites that deliberately supply additional packages;
it inherits the same Ipopt/prebuilt contract.

## Hermetic Verification

Validate a release from a fresh build directory and an MIPSolvers source copy
without `.git` metadata. The following switch prevents any accidental
FetchContent network operation; the build is expected to succeed without it as
well.

```bash
cmake -S . -B build/offline \
  -DCMAKE_BUILD_TYPE=Release \
  -DMIPSOLVERS_SOURCE_DIR=/path/to/MIPSolvers-source-archive \
  -DFETCHCONTENT_FULLY_DISCONNECTED=ON
cmake --build build/offline --parallel
ctest --test-dir build/offline --output-on-failure --parallel 4
```

The configure log must identify Eigen, fmt, nlohmann/json, Catch2, OpenXLSX,
HiGHS, SCIP, PaPILO, MUMPS/Ipopt when enabled, and SuiteSparse as vendored
sources. A missing required source tree must fail at configure time instead of
triggering a download.

Gurobi detection is enabled by default but remains optional and is never
bundled. Set `GUROBI_HOME` before configuring when it is installed outside the
standard Gurobi locations. At runtime an unavailable/expired licence, failed
environment initialization, or failed solve falls back to packaged HiGHS and
native solvers. The reported solver name and DC OPF `solver_chain` identify the
backend that actually produced the result.

External comparison tools such as GridLAB-D, Julia, OpenDSS, Chromium, and
Playwright are runtime test integrations rather than C++ build dependencies.
Their tests skip when the executable is absent. To enable the OpenDSS bridge,
provide a local DSS C-API bundle and set `HACDCPF_ENABLE_OPENDSS=ON`.

## OPF Linear Solver Selection

The parity OPF IPM supports:

```text
HACDCPF_OPF_LINEAR_SOLVER=auto|dense|mumps|umfpack|klu|eigen
```

`auto` uses dense pivoted LU for small KKT systems and local sparse backends for
larger systems. `umfpack` and `klu` use MIPSolvers' vendored SuiteSparse targets;
`eigen` forces the vendored Eigen SparseLU fallback. Use the environment switch
for diagnostics, not as a stable application API.

## Project Options

| Option | Default | Meaning |
|---|---:|---|
| `HACDCPF_DEPENDENCY_PROFILE` | `portable` | Builds the solver subset needed by the application; `full` also builds MIPSolvers developer targets. |
| `HACDCPF_USE_SUITESPARSE` | `ON` | Uses vendored UMFPACK/KLU; `OFF` selects Eigen SparseLU fallback. |
| `HACDCPF_ENABLE_IPOPT` | macOS/Windows presets `ON`, Linux preset `OFF` | Enables embedded Ipopt; Windows consumes the local oneMKL prebuilt package. |
| `HACDCPF_ENABLE_ETAP` | `ON` | Builds against vendored OpenXLSX. |
| `HACDCPF_ENABLE_OPENDSS` | `OFF` | Requires a separately supplied local DSS C-API. |
| `HACDCPF_ENABLE_NATIVE_ARCH` | `OFF` | Enables host-specific CPU instructions; keep `OFF` for portable binaries. |
| `HACDCPF_USE_GUROBI` | `ON` | Detect and prefer an installed/licensed Gurobi; absence is nonfatal. |
| `HACDCPF_USE_PAPILO` | `ON` | Use MIPSolvers' bundled header-only PaPILO; `minimal` profile or `OFF` uses native presolve. |

Native builds on all three target operating systems remain required before a
release: configuring a Windows preset on macOS does not validate MSVC ABI or
runtime packaging. Inspect final executables with `otool -L`, `ldd`, or
`dumpbin /DEPENDENTS` to identify compiler and platform runtime libraries that
must accompany the deployment.

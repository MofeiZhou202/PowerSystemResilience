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
sequential static PardisoMKL backend. It disables Gurobi discovery so the
resulting executable does not acquire a non-redistributable `gurobi*.dll`
startup dependency; packaged HiGHS and SCIP remain available. The packaging
script copies the redistributable VC++/OpenMP runtime DLLs discovered from the
active Visual Studio installation. `windows-vcpkg-release` remains a
compatibility profile for sites that deliberately supply additional packages;
it inherits the same Ipopt/prebuilt contract.

The raw top-level CMake default is also `HACDCPF_ENABLE_IPOPT=ON` on Windows.
MIPSolvers selects `MIPSOLVERS_IPOPT_LINEAR_SOLVER=pardisomkl` there by default;
the supported Windows contract fails configuration when the packaged oneMKL
surface is incomplete instead of silently disabling Ipopt or MKL.

### Windows binary package

Create the full Windows x64 package only from clean HySim and MIPSolvers
checkouts whose dependency commit matches the CMake pin:

```powershell
powershell -ExecutionPolicy Bypass -File tools/package_windows.ps1
```

Use `-SourceDependencies` to select the `windows-source-release` preset. This
builds the open-source dependencies from the sibling MIPSolvers checkout while
still requiring its explicitly staged sequential oneMKL and zlib roots; it does
not prepend a machine-global `C:/vcpkg` prefix to `CMAKE_PREFIX_PATH` and it
does not enable CPLEX.

The script configures and builds `windows-msvc-release`, runs the solver
capability package gate, audits `run_gui_server.exe` with
`dumpbin /DEPENDENTS`, and stages `bin/`, `web/`, `data/`, `external_data/`,
documentation, third-party notices, launch scripts, and `BUILD_INFO.txt`. It
then starts the staged binary from the package directory, verifies the full
edition API and `/xjtu/` frontend, writes a per-file SHA-256 manifest, and
creates `dist/HySim-Windows-x64.zip` plus its SHA-256 sidecar. Extract the ZIP
and run `Start-HySim.cmd`; no source checkout is required at runtime. The
package gate is deliberately narrower than the full CTest sweep; consult the
living development status for current numerical and external-fixture failures.

After packaging, independently verify the archive from a new path containing
spaces and an isolated `PATH`:

```powershell
powershell -ExecutionPolicy Bypass -File tools/verify_windows_release.ps1
```

The verifier checks the ZIP sidecar and every manifest entry, starts only the
extracted executable plus Windows system DLLs, loads the full GUI asset set and
an internal hybrid case, and runs AC Newton PF. The retained extraction folder
contains `verification.json` and server logs for release inspection. This gate
does not replace the complete Windows CTest run.

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

Gurobi detection is enabled by the raw top-level default but remains optional
and is never bundled. The distributable Windows preset explicitly disables it
to avoid a loader-time dependency on a site-local DLL. Custom source builds may
set `GUROBI_HOME` and enable it; once linked, the corresponding Gurobi runtime
must be installed under its own licence. Solver-level initialization or solve
failure can fall back to packaged HiGHS and native solvers, but a missing DLL
cannot be handled after Windows loader failure. The reported solver name and DC
OPF `solver_chain` identify the backend that actually produced the result.

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
| `HACDCPF_ENABLE_IPOPT` | macOS/Windows default and presets `ON`, Linux preset `OFF` | Enables embedded Ipopt; Windows requires the local sequential oneMKL/PardisoMKL prebuilt package. |
| `HACDCPF_ENABLE_ETAP` | `ON` | Builds against vendored OpenXLSX. |
| `HACDCPF_ENABLE_OPENDSS` | `OFF` | Requires a separately supplied local DSS C-API. |
| `HACDCPF_ENABLE_NATIVE_ARCH` | `OFF` | Enables host-specific CPU instructions; keep `OFF` for portable binaries. |
| `HACDCPF_USE_GUROBI` | raw default `ON`; Windows distribution preset `OFF` | Detect and prefer an installed/licensed Gurobi in custom builds; the distributable preset excludes its runtime DLL dependency. |
| `HACDCPF_USE_PAPILO` | `ON` | Use MIPSolvers' bundled header-only PaPILO; `minimal` profile or `OFF` uses native presolve. |

Native builds on all three target operating systems remain required before a
release: configuring a Windows preset on macOS does not validate MSVC ABI or
runtime packaging. Inspect final executables with `otool -L`, `ldd`, or
`dumpbin /DEPENDENTS` to identify compiler and platform runtime libraries that
must accompany the deployment.

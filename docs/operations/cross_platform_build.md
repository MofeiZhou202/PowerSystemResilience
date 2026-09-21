> Documentation Sync (2026-09-19)
> Scope: offline source builds on macOS, Linux, and Windows.
> Status: implementation-backed reference.
> Source of truth: `CMakeLists.txt`, `CMakePresets.json`,
> `cmake/Dependencies.cmake`, and `cmake/MIPSolvers.lock.json` override this
> document if they diverge.

# Cross-Platform Offline Build

The normal build does not download dependencies. The locked in-repository
`MIPSolvers/` import supplies complete local source copies of Eigen 3.4.1, fmt,
nlohmann/json, HiGHS, SCIP, MUMPS, Ipopt, SuiteSparse, Catch2, PaPILO 3.0.0,
and PaPILO's required Boost headers. This repository supplies OpenXLSX for ETAP
Excel I/O. Eigen includes both the core tree and `unsupported/`, including
`unsupported/Eigen/MatrixFunctions`.

## Locked Source Layout

The default and release layout is:

```text
PowerSystemResilience/
  MIPSolvers/
  cmake/MIPSolvers.lock.json
```

The import is locked to upstream `Matrixeigs/MIPSolvers`, branch `windows`,
commit `c6f77f297350b357ff30cc96d9234b2031fd316c`, with the complete imported Git
tree recorded in the lock. CMake verifies selected source hashes even though the
prefixed import has no nested `.git`; release packaging additionally verifies the
committed `HEAD:MIPSolvers` tree and rejects pending changes below that prefix.

Developers may explicitly select another checkout:

```bash
cmake -S . -B build/release \
  -DMIPSOLVERS_SOURCE_DIR=/absolute/path/to/MIPSolvers
```

An independent Git checkout must be at the locked commit. Release/CI use also
requires it to be clean. There is no implicit `../MIPSolvers` fallback, so a
machine-local sibling checkout cannot silently change a build.

Do not omit these tracked source paths from an offline package:

```text
MIPSolvers/third_party/eigen/
MIPSolvers/third_party/fmt/
MIPSolvers/third_party/nlohmann_json/
MIPSolvers/third_party/catch2/
MIPSolvers/third_party/papilo/
MIPSolvers/third_party/boost_papilo/
MIPSolvers/highs/  MIPSolvers/scip/  MIPSolvers/mumps/
MIPSolvers/ipopt/  MIPSolvers/suitesparse/
MIPSolvers/third_party/zlib-1.3.1/
third_party/OpenXLSX-master/
```

The Git source tree is complete for source consumption, but generated Windows
`.lib`/`.dll` files are deliberately not tracked. A checkout containing oneMKL
headers, manifests, and hash lists does not by itself contain a linkable oneMKL
bundle. Generate machine-specific artifacts below the ignored
`build/windows-dependencies/` directory; never write them into the locked import.

## Toolchain Prerequisites

CMake 3.20 or newer and a C++20 compiler are required on every platform.
External comparison tools such as GridLAB-D, Julia, OpenDSS, Chromium, and
Playwright are runtime test integrations rather than core C++ build dependencies.

### macOS

Use Xcode Command Line Tools. The embedded Ipopt/MUMPS profile also needs a
Fortran compiler such as `gfortran`.

```bash
cmake --preset macos-release
cmake --build --preset macos-release
ctest --preset macos-release
```

### Linux

Use GCC or Clang with the standard C/C++ tools. The Linux preset keeps embedded
Ipopt off, so its default profile does not require a Fortran compiler.

```bash
cmake --preset linux-release
cmake --build --preset linux-release
ctest --preset linux-release
```

### Windows dependency staging

Use an x64 Native Tools Prompt or Developer PowerShell for Visual Studio 2022.
Install oneMKL or initialize `MKLROOT`, then run from the repository root:

```powershell
powershell -ExecutionPolicy Bypass -File tools/prepare_windows_dependencies.ps1
```

The script:

1. builds zlib from `MIPSolvers/third_party/zlib-1.3.1`;
2. stages sequential oneMKL from `MKLROOT` with MIPSolvers' hash/manifest logic;
3. leaves all output below `build/windows-dependencies/`.

This prepares the source-built preset:

```powershell
cmake --preset windows-source-release
cmake --build --preset windows-source-release
ctest --preset windows-source-release
```

To prepare the complete ABI-specific prebuilt package required by
`windows-msvc-release`, include `-BuildPrebuiltPackage`:

```powershell
powershell -ExecutionPolicy Bypass -File tools/prepare_windows_dependencies.ps1 `
  -BuildPrebuiltPackage
cmake --preset windows-msvc-release
cmake --build --preset windows-msvc-release
ctest --preset windows-msvc-release
```

The generated paths are:

```text
build/windows-dependencies/zlib/
build/windows-dependencies/oneapi-mkl/
build/windows-dependencies/mipsolvers-third-party/
```

`windows-source-release` builds the other open-source dependencies from the
locked source import and consumes staged zlib/oneMKL. `windows-msvc-release`
consumes the complete prebuilt third-party package. Both use `/MD` in Release,
disable Gurobi for redistribution, keep IPO off, and require sequential
PardisoMKL when Ipopt is enabled. Generated packages are ABI-specific and must
not be reused across MSVC toolsets, architectures, runtime-library modes, or
Release/Debug configurations.

### Resilience and Trial editions

Edition build switches are mutually exclusive. The Resilience preset inherits
the source dependency path and enables the fail-closed Resilience profile:

```powershell
cmake --preset windows-resilience-release
cmake --build --preset windows-resilience-release --target run_gui_server
ctest --preset windows-resilience-release -L edition --output-on-failure
```

The dedicated Resilience package uses an explicit resource/runtime-DLL allowlist
and clean extracted-package verifier:

```powershell
powershell -ExecutionPolicy Bypass -File tools/package_resilience_windows.ps1
```

The script requires the locked in-repository source dependency, builds and runs
the edition unit/API gates, rejects disabled-only resources and non-allowlisted
DLLs, writes a manifest plus ZIP SHA-256, and invokes
`tools/verify_resilience_windows_release.ps1` under an isolated `PATH`. The
scripts define the package contract; successful configuration or their presence
alone does not prove that a Windows archive passed. Consult the
[Resilience Edition contract](resilience_edition_design.md) and living status
for executed evidence. The existing Trial build remains:

```powershell
cmake --preset windows-trial-release
cmake --build --preset windows-trial-release --target run_gui_server
ctest --preset windows-trial-release -L trial --output-on-failure
```

### Full-edition Windows binary package

The generic command below creates the Full Edition package; it is not the
Resilience packaging path. Create it only from a clean repository whose committed
`MIPSolvers/` subtree matches the lock:

```powershell
powershell -ExecutionPolicy Bypass -File tools/package_windows.ps1
```

Use `-SourceDependencies` to select `windows-source-release`. The script verifies
the lock/committed tree, selected dependency mode, and release invariants; builds
and tests the server; audits `run_gui_server.exe` with `dumpbin /DEPENDENTS`;
stages data, GUI, documentation, runtime DLLs, and licenses; runs the staged
server independently; and writes a per-file manifest plus ZIP SHA-256.

After packaging, verify the archive from a fresh path containing spaces and an
isolated `PATH`:

```powershell
powershell -ExecutionPolicy Bypass -File tools/verify_windows_release.ps1
```

This package gate is narrower than the complete Windows CTest suite. Consult the
living development status before making release claims.

## Hermetic Verification

A source archive without nested MIPSolvers Git metadata remains verifiable via
the dependency lock's source hashes. Use a fresh build directory and disable
FetchContent network access:

```bash
cmake -S . -B build/offline \
  -DCMAKE_BUILD_TYPE=Release \
  -DFETCHCONTENT_FULLY_DISCONNECTED=ON
cmake --build build/offline --parallel
ctest --test-dir build/offline --output-on-failure --parallel 4
```

The configure log must identify the locked in-repository import and local
vendored dependencies. Missing or modified verification files fail configuration
rather than triggering a download.

## OPF Linear Solver Selection

The parity OPF IPM supports:

```text
HACDCPF_OPF_LINEAR_SOLVER=auto|dense|mumps|umfpack|klu|eigen
```

`auto` uses dense pivoted LU for small KKT systems and local sparse backends for
larger systems. `umfpack` and `klu` use MIPSolvers' vendored SuiteSparse targets;
`eigen` forces the vendored Eigen SparseLU fallback. This environment switch is
a diagnostic control, not a stable application API.

## Project Options

| Option | Default | Meaning |
|---|---:|---|
| `HACDCPF_DEPENDENCY_PROFILE` | `portable` | Builds the application solver subset; `full` also builds MIPSolvers developer targets. |
| `HACDCPF_TRIAL_EDITION` | `OFF` | Enables the Trial capability profile. Mutually exclusive with Resilience. |
| `HACDCPF_RESILIENCE_EDITION` | `OFF` | Enables the Resilience capability profile. Mutually exclusive with Trial. |
| `HACDCPF_USE_SUITESPARSE` | `ON` | Uses vendored UMFPACK/KLU; `OFF` selects Eigen SparseLU. |
| `HACDCPF_ENABLE_IPOPT` | macOS/Windows default `ON`; Linux preset `OFF` | Enables embedded Ipopt; supported Windows builds require staged sequential oneMKL/PardisoMKL. |
| `HACDCPF_ENABLE_ETAP` | `ON` | Builds against vendored OpenXLSX. |
| `HACDCPF_ENABLE_OPENDSS` | `OFF` | Requires an explicitly supplied local DSS C-API. |
| `HACDCPF_ENABLE_NATIVE_ARCH` | `OFF` | Enables host CPU instructions; keep off for portable binaries. |
| `HACDCPF_USE_GUROBI` | raw default `ON`; Windows release presets `OFF` | Custom builds may use an installed licensed Gurobi; it is not redistributed. |
| `HACDCPF_USE_PAPILO` | `ON` | Uses MIPSolvers' header-only PaPILO; `OFF` uses native presolve. |

Native builds are still required on every target operating system. Use
`otool -L`, `ldd`, or `dumpbin /DEPENDENTS` to audit final runtime dependencies.

# Build and Deployment Guide

MIPSolvers supports macOS, Linux, and Windows. The default build is designed
to work without network access once the repository has been copied to the
build machine.

Do not configure in the source root. Use a separate build directory.

## Dependency model

The following sources are part of the repository and are built locally:

- HiGHS, SCIP, Ipopt, MUMPS 5.7.3, and SuiteSparse
- Eigen 3.4.1, fmt 12.1.0, nlohmann/json 3.11.3, and Catch2 3.7.1
- reference BLAS and LAPACK 3.12.1 as the final numeric fallback

The configure step does not download dependencies. A missing vendored source
tree is a fatal checkout error. PaPILO is disabled by default because its
system packages pull a large shared-library dependency chain.

BLAS/LAPACK selection is:

- macOS: Accelerate, then the vendored reference implementation
- Linux: a system BLAS/LAPACK, then the vendored reference implementation
- Windows: a discoverable system BLAS/LAPACK; MKL is used by the default
  Windows Ipopt profile. The vendored implementation can be forced when a
  supported Fortran compiler is available.

The reference BLAS/LAPACK is intended as a reliable offline fallback. Use
OpenBLAS, MKL, or another optimized implementation for production workloads
where numeric-kernel performance matters.

## Prerequisites

All platforms require CMake 3.20 or newer and a C++20 compiler.

### macOS

Install Xcode command-line tools and a Fortran compiler. For example:

```bash
xcode-select --install
brew install cmake ninja gcc libomp
```

Accelerate is part of macOS. `libomp` is optional; without it the numerical
kernels run serially.

### Linux

For Ubuntu or Debian:

```bash
sudo apt update
sudo apt install -y build-essential cmake ninja-build gfortran
```

System BLAS/LAPACK packages are optional because the repository contains the
reference fallback. For faster production builds, install an optimized BLAS:

```bash
sudo apt install -y libopenblas-dev liblapack-dev
```

### Windows

The tested native toolchain profile is Visual Studio 2022 x64. Start from an
`x64 Native Tools Command Prompt` or a PowerShell session initialized by
`VsDevCmd.bat`. Do not mix MSVC libraries with a MinGW C++ build.

Two Windows profiles are supported:

1. MSVC plus oneAPI MKL: the default and lowest-risk profile. Embedded Ipopt
   uses PardisoMKL and does not build MUMPS.
2. A source-only numeric profile: set the Ipopt backend to `mumps`, force the
   vendored reference BLAS/LAPACK, and provide Intel oneAPI `ifx`/`ifort` or a
   consistent MinGW `gfortran` toolchain.

For the default profile, initialize oneAPI before configuring so `MKLROOT` is
set and the MKL libraries are visible:

```powershell
cmd /k '"C:\Program Files (x86)\Intel\oneAPI\setvars.bat" intel64'
```

The `windows-msvc-release` preset contains no machine-specific compiler or SDK
paths. It uses the compiler, Ninja, Windows SDK, and oneAPI environment from
the current shell.

## Build from source

### macOS and Linux

```bash
cmake -S . -B build/release \
  -DCMAKE_BUILD_TYPE=Release \
  -DMIPSOLVERS_USE_PREBUILT_THIRD_PARTY=OFF \
  -DMIPSOLVERS_USE_PAPILO=OFF
cmake --build build/release --parallel
ctest --test-dir build/release -L unit --output-on-failure --parallel 4
```

To require the bundled reference BLAS/LAPACK instead of Accelerate or a system
implementation:

```bash
cmake -S . -B build/reference-blas \
  -DCMAKE_BUILD_TYPE=Release \
  -DMIPSOLVERS_USE_PREBUILT_THIRD_PARTY=OFF \
  -DMIPSOLVERS_FORCE_VENDORED_BLAS=ON \
  -DMIPSOLVERS_USE_PAPILO=OFF
cmake --build build/reference-blas --parallel
```

`MIPSOLVERS_USE_VENDORED_BLAS=ON` allows the fallback and is the default.
`MIPSOLVERS_FORCE_VENDORED_BLAS=ON` skips system detection and is the option to
use for fallback validation.

### Windows default profile

```powershell
cmake --preset windows-msvc-release
cmake --build --preset windows-msvc-release
ctest --preset windows-msvc-release -L unit
```

The equivalent explicit configuration is:

```powershell
cmake -S . -B build/windows-msvc -G "Ninja Multi-Config" `
  -DMIPSOLVERS_IPOPT_LINEAR_SOLVER=pardisomkl `
  -DMIPSOLVERS_USE_PAPILO=OFF `
  -DMIPSOLVERS_USE_PREBUILT_THIRD_PARTY=OFF
cmake --build build/windows-msvc --config Release --parallel 8
ctest --test-dir build/windows-msvc -C Release -L unit --output-on-failure
```

### Windows source-only numeric profile

Run from a shell where the selected Fortran compiler is available. For Intel
oneAPI, pass `ifx.exe` explicitly when auto-detection is insufficient:

```powershell
cmake -S . -B build/windows-source -G "Ninja Multi-Config" `
  -DMIPSOLVERS_FORTRAN_COMPILER="C:/Program Files (x86)/Intel/oneAPI/compiler/latest/bin/ifx.exe" `
  -DMIPSOLVERS_IPOPT_LINEAR_SOLVER=mumps `
  -DMIPSOLVERS_FORCE_VENDORED_BLAS=ON `
  -DMIPSOLVERS_USE_PAPILO=OFF `
  -DMIPSOLVERS_USE_MKL=OFF
cmake --build build/windows-source --config Release --parallel 8
```

This path is supported by the CMake logic but must be validated on the exact
Visual Studio, oneAPI, Windows SDK, and architecture combination used for the
deployment package.

## Precompiled third-party package

Third-party libraries can be compiled once and reused by development builds.
The generated package is local to its compiler, SDK, architecture, runtime,
and configuration; it is not a universal binary cache.

### macOS and Linux

```bash
third_party/build_third_party.sh --jobs 8 --build-type Release
```

The default output is:

```text
third_party/build/
third_party/install/
```

Both directories are ignored by git. Use `--fresh` to discard only the local
third-party build directory before rebuilding.

### Windows

```powershell
third_party/build_third_party.ps1 -Jobs 8 -BuildType Release -Fresh
```

Run the script from the same initialized Visual Studio/oneAPI environment that
will be used for the main build.

### Reuse policy

`MIPSOLVERS_USE_PREBUILT_THIRD_PARTY` accepts:

- `AUTO` (default): reuse a compatible local manifest, otherwise warn and
  build the vendored sources in-tree
- `ON`: require a compatible package and fail if it is absent or incompatible
- `OFF`: always build dependencies in-tree

After creating the package:

```bash
cmake -S . -B build/dev \
  -DCMAKE_BUILD_TYPE=Release \
  -DMIPSOLVERS_USE_PREBUILT_THIRD_PARTY=ON
cmake --build build/dev --parallel
ctest --test-dir build/dev -L unit --output-on-failure --parallel 4
```

The manifest records dependency availability, public compile definitions, and
toolchain identity. `AUTO` falls back to the in-tree build on a mismatch;
`ON` reports the mismatch as a configuration error. A multi-config generator
is restricted to the configuration installed in the package, preventing a
Release dependency package from being linked into a Debug MSVC build.

## Important CMake options

| Option | Default | Purpose |
|---|---:|---|
| `MIPSOLVERS_BUILD_TESTS` | `ON` | Build unit and integration tests |
| `MIPSOLVERS_THIRD_PARTY_ONLY` | `OFF` | Build/install dependencies only |
| `MIPSOLVERS_USE_PREBUILT_THIRD_PARTY` | `AUTO` | `AUTO`, `ON`, or `OFF` reuse policy |
| `MIPSOLVERS_USE_VENDORED_BLAS` | `ON` | Allow reference BLAS/LAPACK fallback |
| `MIPSOLVERS_FORCE_VENDORED_BLAS` | `OFF` | Force reference BLAS/LAPACK |
| `MIPSOLVERS_FORCE_BUILD_MUMPS` | `ON` | Build vendored MUMPS instead of Homebrew MUMPS |
| `MIPSOLVERS_STATIC_LIBGFORTRAN` | `OFF` | Static GNU Fortran runtime flags on Linux |
| `MIPSOLVERS_IPOPT_LINEAR_SOLVER` | platform dependent | `mumps`, or `pardisomkl` on Windows |
| `MIPSOLVERS_USE_PAPILO` | `OFF` | Opt in to system PaPILO |
| `MIPSOLVERS_USE_SYSTEM_FMT` | `OFF` | Opt in to system fmt |
| `MIPSOLVERS_USE_OPENMP` | `ON` | Enable OpenMP when detected |

## Runtime deployment

### macOS

Accelerate, `libc++`, and `libSystem` are system libraries. If OpenMP is
enabled, ship a compatible `libomp.dylib` with the application and fix its
install name/RPATH, or install `libomp` on the target. Static OpenMP linking is
not generally supported on macOS.

The vendored MUMPS build needs a Fortran runtime. The build prefers static GCC
Fortran runtime archives on macOS; if they are unavailable, CMake emits a
warning and the final executable depends on the matching `libgfortran` and
`libquadmath` dylibs. Check the final executable, not only static archives:

```bash
otool -L path/to/application
```

### Linux

GCC supplies `libgomp` and the GNU Fortran runtime. For a more self-contained
deployment, enable `MIPSOLVERS_STATIC_LIBGFORTRAN=ON`; confirm that the target
distribution permits static `libgcc`/`libgfortran` linking. Inspect the final
ELF dependencies with:

```bash
ldd path/to/application
```

### Windows

Use one C/C++ runtime model consistently. The presets use the dynamic MSVC
runtime (`/MD` for Release and `/MDd` for Debug). Ship the corresponding MSVC
redistributable. A oneAPI MKL build also needs the oneAPI runtime DLLs selected
by its link model; use `dumpbin /DEPENDENTS application.exe` to enumerate the
actual package requirements.

Prebuilt third-party `.lib` files must be regenerated when changing MSVC
toolset, target architecture, oneAPI compiler/runtime, or MSVC runtime model.

## Install and consume

Install a normal in-tree build with:

```bash
cmake --install build/release --prefix /path/to/prefix
```

A consumer should use the exported target:

```cmake
find_package(mipsolvers CONFIG REQUIRED)
target_link_libraries(my_app PRIVATE mipsolvers::mipsolvers)
```

The installed package includes vendored Eigen and nlohmann/json headers and,
when vendored fmt/SuiteSparse targets were built, exports those targets. It
does not unconditionally search Homebrew paths.

For source integration:

```cmake
set(MIPSOLVERS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MIPSOLVERS_BUILD_SCUC OFF CACHE BOOL "" FORCE)
add_subdirectory(path/to/MIPSolvers EXCLUDE_FROM_ALL)
target_link_libraries(my_app PRIVATE mipsolvers::mipsolvers)
```

## Verification checklist

For a release candidate:

1. Configure in a new build directory with
   `MIPSOLVERS_USE_PREBUILT_THIRD_PARTY=OFF` and confirm no download activity.
2. Build and run `ctest -L unit` with PaPILO disabled.
3. Build and test once with `MIPSOLVERS_FORCE_VENDORED_BLAS=ON`.
4. Build the third-party-only package, then configure another new directory
   with `MIPSOLVERS_USE_PREBUILT_THIRD_PARTY=ON` and run the unit tests.
5. Inspect the final executable with `otool -L`, `ldd`, or
   `dumpbin /DEPENDENTS`; static-library inspection alone is insufficient.
6. Repeat the Windows build on the deployment toolchain. Cross-platform CMake
   configuration from macOS or Linux does not validate MSVC/ifx ABI behavior.

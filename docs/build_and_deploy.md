# Build and Deployment Guide

This document covers how to compile MIPSolvers from source, install the
libraries, and consume them in other projects on macOS, Linux, and Windows.

---

## Requirements

| Tool | Minimum version | Notes |
|---|---|---|
| CMake | 3.20 | |
| C++ compiler | C++20 support | Clang 14+, GCC 12+, MSVC 2022 (19.30+) |
| Ninja or GNU Make | any | optional but recommended |

All three embedded solvers (HiGHS 1.14.0, SCIP, Ipopt) are compiled from
source inside the repository. No system solver installations are required.

### Optional libraries (auto-detected via `find_package`)

| Library | Feature enabled when found |
|---|---|
| SuiteSparse (UMFPACK + KLU) | Sparse LU backends for IPM/NLE |
| SuperLU | Alternative sparse LU |
| Intel MKL + PARDISO | MKL sparse direct solver |
| PaPILO | Advanced MILP presolve |
| Gurobi 9.5–13.0 | External MIP reference solver |

---

## Platform-specific prerequisites

### macOS

```bash
# Xcode command-line tools (provides clang + libc++)
xcode-select --install

# CMake and optional accelerator libraries via Homebrew
brew install cmake ninja suite-sparse
```

The Apple Accelerate framework (`-framework Accelerate`) is linked
automatically by CMake when building on macOS.

Ipopt is built from the embedded `ipopt/` source by default on macOS
(`MIPSOLVERS_BUILD_LOCAL_IPOPT=ON`).

### Linux (Ubuntu / Debian)

```bash
sudo apt update
sudo apt install -y \
    build-essential cmake ninja-build \
    libsuitesparse-dev libsuperlu-dev \
    libeigen3-dev libfmt-dev nlohmann-json3-dev
```

Ipopt is **not** built by default on Linux (the embedded build requires
Fortran support and additional LAPACK/BLAS dependencies). Either install a
system Ipopt or explicitly enable the embedded build:

```bash
sudo apt install -y gfortran liblapack-dev libblas-dev
# then pass -DMIPSOLVERS_BUILD_LOCAL_IPOPT=ON to cmake
```

### Windows (MSVC)

Open an **x64 Native Tools Command Prompt for VS 2022** (or set up the
MSVC environment via `vcvars64.bat`), then use CMake's Visual Studio
generator or Ninja:

```powershell
# Install optional deps via vcpkg (recommended)
vcpkg install suitesparse:x64-windows nlohmann-json:x64-windows fmt:x64-windows eigen3:x64-windows

# Install Intel oneAPI MKL when building embedded Ipopt on Windows.

cmake -S . -B build -A x64 `
  -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake"
```

Embedded Ipopt uses MKL Pardiso on Windows (`MIPSOLVERS_IPOPT_LINEAR_SOLVER=pardisomkl`)
and does not require a Fortran compiler. The older MUMPS backend is still
available with `-DMIPSOLVERS_IPOPT_LINEAR_SOLVER=mumps`, but that path requires
Fortran.

The Windows deployment preset disables PaPILO (`MIPSOLVERS_USE_PAPILO=OFF`)
because some PaPILO package configs enable Fortran during discovery. Leave it
off unless you specifically need that presolve backend.

---

## Build

### 1. Configure

The examples below build a Release binary into `build/`.
Adjust `-DCMAKE_INSTALL_PREFIX` to your target install root.

**macOS / Linux — library only (fastest, for use in other projects):**

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DMIPSOLVERS_BUILD_TESTS=OFF \
  -DMIPSOLVERS_BUILD_SCUC=OFF \
  -DMIPSOLVERS_BUILD_SCUC_CASE_BUILDER=OFF \
  -DCMAKE_INSTALL_PREFIX="$HOME/.local"
```

Add `-DMIPSOLVERS_ENABLE_NATIVE_ARCH=ON` to use `-march=native` for a
host-tuned binary. Omit this flag if the binary will run on other machines.

**macOS / Linux — full build with tests, tools, and SCUC:**

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DMIPSOLVERS_BUILD_TESTS=ON \
  -DMIPSOLVERS_BUILD_SCUC=ON \
  -DMIPSOLVERS_BUILD_SCUC_CASE_BUILDER=ON \
  -DCMAKE_INSTALL_PREFIX="$HOME/.local"
```

**Windows — full build:**

```powershell
cmake -S . -B build -A x64 `
  -DMIPSOLVERS_BUILD_TESTS=ON `
  -DMIPSOLVERS_BUILD_SCUC=ON `
  -DMIPSOLVERS_BUILD_SCUC_CASE_BUILDER=OFF `
  -DMIPSOLVERS_BUILD_PYTHON=OFF `
  -DMIPSOLVERS_ENABLE_WERROR=OFF
```

### 2. Compile

```bash
# macOS / Linux
cmake --build build --parallel $(nproc 2>/dev/null || sysctl -n hw.logicalcpu)

# macOS / Linux — library target only
cmake --build build --target mipsolvers --parallel $(sysctl -n hw.logicalcpu)

# Windows
cmake --build build --config Release --parallel 8
```

### 3. Run tests

```bash
ctest --test-dir build -C Release --output-on-failure --parallel 4
```

Run only fast unit tests:

```bash
ctest --test-dir build -C Release -L unit --output-on-failure --parallel 4
```

### 4. Install

```bash
cmake --install build
# or, specifying the prefix at install time:
cmake --install build --prefix /path/to/target
```

Installed layout:

```
<prefix>/
  include/mipsolvers/     ← all public headers
  lib/libmipsolvers.a
  lib/libmipsolvers_hfactor.a
```

---

## CMake options reference

| Option | Default | Description |
|---|---|---|
| `CMAKE_BUILD_TYPE` | `Release` | `Release`, `Debug`, or `RelWithDebInfo` |
| `MIPSOLVERS_BUILD_TESTS` | `ON` | Build Catch2 unit / integration tests |
| `MIPSOLVERS_BUILD_SCUC` | `ON` | Build SCUC solver (`scuc_solve`) |
| `MIPSOLVERS_BUILD_SCUC_CASE_BUILDER` | `ON` | Build `scuc_case_builder` tool |
| `MIPSOLVERS_BUILD_PYTHON` | `OFF` | Build pybind11 Python extension (see below) |
| `MIPSOLVERS_BUILD_HFACTOR` | `ON` | Build HiGHS HFactor static library |
| `MIPSOLVERS_BUILD_EMBEDDED_HIGHS` | `ON` | Compile HiGHS from `highs/` |
| `MIPSOLVERS_BUILD_EMBEDDED_SCIP` | `ON` | Compile SCIP from `scip/` |
| `MIPSOLVERS_BUILD_LOCAL_IPOPT` | `ON` | Compile Ipopt from `ipopt/` |
| `MIPSOLVERS_IPOPT_LINEAR_SOLVER` | `pardisomkl` on Windows, `mumps` elsewhere | Embedded Ipopt linear solver backend |
| `MIPSOLVERS_ENABLE_NATIVE_ARCH` | `OFF` | Add `-march=native` (host-only builds) |
| `MIPSOLVERS_ENABLE_WERROR` | `OFF` | Treat compiler warnings as errors |
| `MIPSOLVERS_EIGEN_VECTORIZE` | `ON` | Enable Eigen SIMD intrinsics |
| `MIPSOLVERS_EIGEN_MAX_ALIGN_BYTES` | `32` | Eigen alignment (16/32/64) |
| `MIPSOLVERS_USE_SUITESPARSE` | `ON` | Auto-detect and use SuiteSparse |
| `MIPSOLVERS_USE_SUPERLU` | `ON` | Auto-detect and use SuperLU |
| `MIPSOLVERS_USE_PAPILO` | `ON` | Auto-detect and use PaPILO presolve |

---

## Produced artifacts

| File (relative to build dir) | Description |
|---|---|
| `libmipsolvers.a` | Main static library — link this in consumer projects |
| `src/engine/kernel/linear_algebra/highs_factor/libmipsolvers_hfactor.a` | HiGHS HFactor linear algebra kernel |
| `lib/libhighs.a` | Embedded HiGHS 1.14.0 |
| `libipopt_local.a` | Embedded Ipopt (macOS / when enabled) |
| `_deps/embedded_scip/libscip.a` | Embedded SCIP |
| `milp_benchmark_runner` | Benchmark executable |
| `scuc_solve` | SCUC command-line solver |
| `scuc_case_builder` | SCUC case file builder |

---

## Using MIPSolvers in another project

### Option A — `add_subdirectory` (source integration)

The cleanest approach if both projects are in the same repository tree:

```cmake
# In the consumer CMakeLists.txt
add_subdirectory(path/to/MIPSolvers EXCLUDE_FROM_ALL)
target_link_libraries(my_app PRIVATE mipsolvers::mipsolvers)
```

Set `-DMIPSOLVERS_BUILD_TESTS=OFF -DMIPSOLVERS_BUILD_SCUC=OFF` to suppress
building the executables and tests inside the subdirectory:

```cmake
set(MIPSOLVERS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MIPSOLVERS_BUILD_SCUC  OFF CACHE BOOL "" FORCE)
add_subdirectory(path/to/MIPSolvers EXCLUDE_FROM_ALL)
```

### Option B — installed prefix (recommended for separate projects)

After running `cmake --install build --prefix /path/to/prefix`:

```cmake
# Point CMake at the install prefix
list(APPEND CMAKE_PREFIX_PATH "/path/to/prefix")

# If an exported Config file is present:
find_package(mipsolvers REQUIRED)
target_link_libraries(my_app PRIVATE mipsolvers::mipsolvers)
```

If no CMake Config file was exported (manual library linking):

```cmake
target_include_directories(my_app PRIVATE "/path/to/prefix/include")

target_link_libraries(my_app PRIVATE
  "/path/to/prefix/lib/libmipsolvers.a"
  "/path/to/prefix/lib/libmipsolvers_hfactor.a"
  "/path/to/prefix/lib/libhighs.a"       # required — embedded in mipsolvers
  "/path/to/prefix/lib/libscip.a"        # required — embedded in mipsolvers
  Eigen3::Eigen                          # public dependency
  fmt::fmt                               # public dependency
  nlohmann_json::nlohmann_json           # public dependency
)

# macOS only
if(APPLE)
  target_link_libraries(my_app PRIVATE "-framework Accelerate")
endif()
```

> `libhighs.a`, `libscip.a`, and `libipopt_local.a` are `PRIVATE`
> dependencies of `libmipsolvers.a`. When consuming the `.a` directly
> (without an exported CMake target) they must be listed explicitly because
> static libraries do not propagate their transitive dependencies.

### Option C — single bundled static archive

The build produces a single self-contained archive that merges
`libmipsolvers.a` with every static dependency built in-tree (HiGHS, SCIP,
Ipopt, MUMPS when selected, the HiGHS-factor kernel, and the static LUSOL archive). This is
controlled by `MIPSOLVERS_BUILD_BUNDLED_ARCHIVE` (default `ON`) and produced by
the `mipsolvers_bundled` target:

```bash
cmake --build <build_dir> --target mipsolvers_bundled -j8
# → <build_dir>/libmipsolvers_bundled.a   (one archive, ~36 MB on macOS)
```

A consumer then links a single archive plus the external *shared* libraries
(these cannot be merged into a static archive):

```cmake
target_include_directories(my_app PRIVATE "/path/to/prefix/include")
target_link_libraries(my_app PRIVATE
  "/path/to/libmipsolvers_bundled.a"     # everything we build, in one .a

  # External shared libraries the consumer must still provide:
  /opt/homebrew/opt/suite-sparse/lib/libumfpack.dylib
  /opt/homebrew/opt/suite-sparse/lib/libcholmod.dylib
  /opt/homebrew/opt/suite-sparse/lib/libamd.dylib
  /opt/homebrew/opt/suite-sparse/lib/libcolamd.dylib
  /opt/homebrew/opt/suite-sparse/lib/libklu.dylib
  /opt/homebrew/opt/suite-sparse/lib/libsuitesparseconfig.dylib
  /opt/homebrew/opt/openblas/lib/libopenblas.dylib
  fmt::fmt
  /opt/homebrew/lib/libgmp.dylib /opt/homebrew/lib/libgmpxx.dylib
  /opt/homebrew/lib/libtbb.dylib
  # Boost (pulled in by SCIP): container iostreams program_options random regex serialization
  # Gurobi (only if the Gurobi adapter is compiled in):
  #   /Library/gurobi<ver>/macos_universal2/lib/libgurobi<ver>.dylib
  -lgfortran -lquadmath                  # macOS/Linux MUMPS/LUSOL Fortran runtime
)
if(APPLE)
  target_link_libraries(my_app PRIVATE "-framework Accelerate")
endif()
```

> The bundled archive contains only the libraries we build ourselves; system
> numeric/shared libraries (SuiteSparse, OpenBLAS, GMP, TBB, Boost, any Fortran
> runtime used by MUMPS/LUSOL, and Accelerate) are intentionally left out and must be supplied by
> the consumer at link time.

### Minimum consumer CMakeLists.txt example

```cmake
cmake_minimum_required(VERSION 3.20)
project(my_project CXX)
set(CMAKE_CXX_STANDARD 20)

# --- build MIPSolvers as a subdirectory ---
set(MIPSOLVERS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MIPSOLVERS_BUILD_SCUC  OFF CACHE BOOL "" FORCE)
add_subdirectory(../MIPSolvers MIPSolvers-build EXCLUDE_FROM_ALL)

add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE mipsolvers::mipsolvers)
```

---

## Python bindings

The Python extension is disabled by default. Enable it with:

```bash
cmake -S . -B build \
  -DMIPSOLVERS_BUILD_PYTHON=ON \
  -DPYTHON_EXECUTABLE=$(which python3)
```

pybind11 must be installed in the target Python environment:

```bash
pip install pybind11
```

The built `.so` / `.pyd` file (e.g. `mipsolvers.cpython-312-darwin.so`) can
be placed directly on `sys.path` or installed via `cmake --install`.

---

## Debug / RelWithDebInfo builds

Replace `-DCMAKE_BUILD_TYPE=Release` with `Debug` or `RelWithDebInfo`:

```bash
cmake -S . -B build-debug \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DMIPSOLVERS_BUILD_TESTS=ON
cmake --build build-debug --parallel
```

`RelWithDebInfo` compiles with `-O2 -g -DNDEBUG` and LTO — useful for
profiling with line information.

---

## Troubleshooting

**HiGHS configure error: `MIPSOLVERS_BUILD_EMBEDDED_HIGHS=ON, but no embedded HiGHS target`**

The `highs/` directory must contain the HiGHS source (at minimum
`CMakeLists.txt` and `HConfig.h.in`). Check that the repository was
cloned with all subdirectories present.

**SCIP configure error: `no embedded SCIP target`**

Same as above — `scip/CMakeLists.txt` must exist.

**`undefined symbol` linking `libmipsolvers.a` from an external project**

Add `libhighs.a` and `libscip.a` explicitly (see Option B above). Static
libraries do not carry transitive link information.

**Windows: `No such file or directory: /dev/null` in tests**

This is a known historical issue fixed in `src/engine/solver/external/adapters.cpp`
and `tests/test_market_simulation.cpp`. The subprocess null device is
selected at compile time (`NUL` on Windows, `/dev/null` elsewhere), and
temporary files use `std::filesystem::temp_directory_path()`.

**macOS: linker warning about duplicate `-framework Accelerate`**

This is harmless. It appears when both `libhighs.a` and `libmipsolvers.a`
pass the flag; the linker deduplicates the framework.

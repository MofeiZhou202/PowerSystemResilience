# MIPSolvers 0.1.0 Windows x64 Production SDK

Relocatable C++ SDK built from commit
`80f49f6a97f628f3663939ab1613d6e7ccaf03c1` on 2026-08-20.

## Supported environment

- Windows x64
- Visual Studio 2022 / MSVC 19.44 or a binary-compatible MSVC toolchain
- Release CRT model: `/MD`
- Runtime prerequisite: Microsoft Visual C++ 2015-2022 Redistributable (x64)

The SDK carries the required static oneMKL 2026.0.1 LP64/sequential libraries,
vendored SuiteSparse/KLU/UMFPACK/CHOLMOD, embedded HiGHS 1.14.0, Eigen, fmt,
and nlohmann_json headers. No separate MKL, Intel OpenMP, ZLIB, SCIP, Ipopt,
MUMPS, or SuperLU DLL is required.

## Consumer usage

```cmake
find_package(mipsolvers 0.1 REQUIRED)
target_link_libraries(your_target PRIVATE mipsolvers::mipsolvers)
```

Configure with the extracted SDK as the package prefix:

```powershell
cmake -S . -B build -G "Ninja Multi-Config" `
  -DCMAKE_PREFIX_PATH=C:\path\to\MIPSolvers-0.1.0-windows-x64-production-80f49f6
cmake --build build --config Release
```

Use the headers and libraries from this package together. Do not mix them with
an older SDK because the sparse-solver virtual interface includes the numeric
refactor capability introduced by this release.

## Build configuration

```text
Generator: Ninja Multi-Config
Compiler: MSVC 19.44.35228
Configuration: Release (/MD)
MIPSOLVERS_BUILD_EMBEDDED_HIGHS=ON
MIPSOLVERS_BUILD_HFACTOR=ON
MIPSOLVERS_USE_PAPILO=ON
MIPSOLVERS_USE_SUITESPARSE=ON
MIPSOLVERS_USE_VENDORED_CHOLMOD=ON
MIPSOLVERS_USE_MKL=ON
MIPSOLVERS_MKL_THREADING=SEQUENTIAL
MIPSOLVERS_USE_OPENMP=OFF
MIPSOLVERS_BUILD_EMBEDDED_SCIP=OFF
MIPSOLVERS_BUILD_LOCAL_IPOPT=OFF
MIPSOLVERS_USE_MUMPS=OFF
MIPSOLVERS_USE_SUPERLU=OFF
MIPSOLVERS_USE_GUROBI=OFF
MIPSOLVERS_ENABLE_NATIVE_ARCH=OFF
MIPSOLVERS_ENABLE_IPO=OFF
```

## Validation

- Clean rebuild: 982/982 build steps completed.
- Release CTest: 19/19 passed in 173.12 seconds.
- External fresh-tree consumer configured using only `CMAKE_PREFIX_PATH`,
  linked successfully, and returned objective `9.000000`.
- `dumpbin /DEPENDENTS`: Windows system DLLs and MSVC runtime only.
- Installed headers and CMake files contain no source, build, or user path.
- Static oneMKL payload: 638,380,710 bytes; nine license files included.
- KLU fixed-pattern benchmark: 0.916629 ms/full factorization versus
  0.218438 ms/refactorization, 4.196x; residual `1.421e-15`, solution delta 0.

The KLU number is a controlled microbenchmark, not an end-to-end NLE claim.
Verify all extracted files against `SHA256SUMS` before use. Redistribution of
bundled third-party components remains subject to the license files shipped in
`share/mipsolvers/licenses` and their upstream license terms.

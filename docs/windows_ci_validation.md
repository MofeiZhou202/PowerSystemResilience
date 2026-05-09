# Windows/CI Regression Validation

This note captures the Windows-side reproduction and validation path for the
cross-platform regressions fixed in:

- `src/engine/solver/external/adapters.cpp`
- `src/engine/kernel/linear_algebra/linear_solver.cpp`
- `tests/test_market_simulation.cpp`
- `tests/test_engine_api.cpp`

## Scope

The regression surface is the shared execution path used by these tests:

- `test_engine_api`
- `test_milp_solver`
- `test_ipm_solver`
- `test_scuc_module`
- `test_market_simulation`

The two concrete Windows-specific failures were:

1. External solver subprocess redirection hardcoded to `/dev/null`.
2. Debug JSON export in `test_market_simulation.cpp` hardcoded to `/tmp`.
3. Empty sparse systems reaching Eigen SparseLU and dividing by zero in
  `SparseLU_Memory.h`.

Windows does not provide either path. The expected fixed behavior is:

- subprocess output is redirected to `NUL` on Windows;
- temporary debug JSON is written via `std::filesystem::temp_directory_path()`.
- empty `0x0` sparse systems are short-circuited before calling Eigen or
  SuiteSparse backends.

## Local Reproduction On Windows

Open an `x64 Native Tools Command Prompt for VS 2022` or PowerShell with the
MSVC toolchain on `PATH`, then run from the repository root.

### 1. Configure

`MIPSOLVERS_BUILD_SCUC=ON` is required because `test_scuc_module` and
`test_market_simulation` are only registered when SCUC is enabled.

```powershell
cmake -S . -B build-win -A x64 `
  -DMIPSOLVERS_BUILD_TESTS=ON `
  -DMIPSOLVERS_BUILD_PYTHON=OFF `
  -DMIPSOLVERS_BUILD_SCUC=ON `
  -DMIPSOLVERS_BUILD_SCUC_CASE_BUILDER=OFF `
  -DMIPSOLVERS_ENABLE_WERROR=OFF
```

### 2. Build The Affected Targets

`test_market_simulation` already links `src/scuc/case_builder.cpp` directly, so
the standalone `scuc_case_builder` executable is not needed for this regression
pass.

```powershell
cmake --build build-win --config Release `
  --target test_engine_api test_milp_solver test_ipm_solver test_scuc_module test_market_simulation `
  --parallel 4
```

### 3. Run The Windows Regression Slice

```powershell
ctest --test-dir build-win --build-config Release `
  -R "test_(engine_api|milp_solver|ipm_solver|scuc_module|market_simulation)" `
  --output-on-failure --parallel 2
```

### 4. Optional: Run The Full Test Suite

```powershell
ctest --test-dir build-win --build-config Release --output-on-failure --parallel 2
```

## Expected Pass Criteria

- Configure and build complete without requiring Gurobi.
- `test_engine_api`, `test_milp_solver`, `test_ipm_solver`,
  `test_scuc_module`, and `test_market_simulation` all pass.
- No failure output contains `/dev/null` or `/tmp` path assumptions.
- `test_market_simulation` writes debug JSON to a valid Windows temp directory
  when those code paths execute.
- `test_engine_api` confirms `EigenSparseLUSolver` accepts empty `0x0` systems
  without crashing.

## CI Translation

The matching GitHub Actions regression job should:

1. run on `windows-latest`;
2. configure with tests enabled, Python disabled, SCUC enabled;
3. build only the five affected test targets;
4. run a targeted `ctest -R` pass for those five tests.

This keeps the Windows job focused on the exact regression surface while still
covering the shared solver-dispatch path that caused the original failures.

## Notes

- This repository can be built without external HiGHS or SCIP executables; the
  embedded source trees are sufficient for CI coverage.
- If VS Code CMake Tools cannot configure the project, use the terminal-based
  `cmake` and `ctest` commands above as the source of truth.
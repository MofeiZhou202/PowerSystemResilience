# Windows/main synchronization, 2026-09-17

Parents: Windows `253d82b0`; main `8fdea5a1`.

## Pre-registered rationale

Model/algorithm: import the main Benders, integer L-shaped, discrete CCG and
polyhedral KKT-oracle implementations under their existing assumptions and
proof-bearing status gates. References: `two_stage_decomposition_design.md`,
Sections 1-3; Van Slyke--Wets (1969), Birge--Louveaux (2011), Chapter 5;
Zeng--Zhao (2013). Preserve the Windows native LP/IPM contracts in
`windows_remediation_2026-09-11.md` (R4) and
`general_solver_performance_program_2026-09-13.md` (R1-R4).

Claim: expose main's new functionality on Windows without relaxing numerical
acceptance gates or replacing Windows resource accounting and direct-IPM policy.

Cost model: decomposition retains one master solve and scenario recourse solves
per major iteration, plus sparse cut/model assembly. This merge adds no
algorithmic work relative to those upstream implementations. Deadline checks
and timing samples use the monotonic host clock. No wall-time speedup is claimed.

Prediction: zero changed mathematical tolerances; all registered CTest targets
pass; existing decomposition oracle comparisons satisfy their 1e-6 checks;
zero documentation anchor failures. Tests, rather than benchmark timings, are
the acceptance metric for this synchronization.

Assumptions: existing MSVC x64 Release/MKL configuration, finite valid models,
backend proof certificates and decomposition domain restrictions as specified
by the imported design. Optional backends are checked only when configured.

Validation fixed before implementation:

- `cmake --build build/windows-msvc-release --config Release --parallel 4`
- `ctest --test-dir build/windows-msvc-release -C Release --output-on-failure -j 2`
- `python tools/doc_anchor_check.py`
- Existing Python R3 evidence validator tests and Windows Unicode worker smoke.

## Integration mismatch investigation

The first build rejected duplicate `HighsAdapter::solve_lp` and
`HighsAdapter::solve_milp` context overloads. Git had automatically retained
both the Windows implementations (forwarding time, threads and seed) and main's
older context implementations (forwarding time only). This is implementation
infidelity in the merge, not a numerical-model discrepancy. Remove only the
redundant upstream definitions and retain the complete Windows resource
forwarding; retain main's explicit rejection of invalid HiGHS deadlines.
No test threshold or numerical constant changes are required.


The first CTest run exposed ABI-inconsistent incremental objects: native IPM,
B&C and NLP benchmarks crashed, while registry and dual-simplex checks passed.
`ninja -t deps` reports zero recorded dependencies for 12 repository-owned
Release objects, including `ipm_solver.cpp.obj` and `branch_and_cut.cpp.obj`.
The native IPM object still dates from September 13 despite the September 17
`SolveStats` layout change. In contrast, rebuilt `ipm_lp_solver.cpp.obj` records
265 headers, including `solver_adapter.hpp`. Thus some callers used the new
layout while these old objects used the previous layout. This is a build-cache
implementation-fidelity defect; a compiler-language mismatch is a possible
origin of the missing metadata, but has not been established.

Invalidate only these generated objects, rebuild with the current working
MSVC/Ninja header scanner, verify dependencies are now populated, and repeat
the full CTest protocol. No solver formula or tolerance changes are warranted.
The interrupted failing run is retained as `reports/main-sync-20260917-ctest-first.log`.


## Merge decisions

- Preserve Windows `SolveContext` automatic thread resolution, memory accounting,
  deterministic seed forwarding, session APIs and original-coordinate IPM gates.
- Import upstream portfolio timing and cancellation joins while retaining the
  direct-IPM/no-presolve Windows portfolio policy.
- Keep both release CI gates and upstream hard-deadline/Unicode-path probes.
- Import main's implied-bound activity identity fix, large-model deadline size
  gates and explicit concurrent-tree opt-in with their existing regression tests.
- Archive imported design records under `docs/archive/`, repair their source
  citations, and document public additions in the maintained manual.
- Rephrase workflow prose and construct its unchanged marker regex from shell
  string fragments so the required added-line guard does not match its own
  instructions. The prohibition and pattern semantics remain unchanged.


After rebuilding, the native IPM object records 212 project dependencies and
B&C records 392. Four standalone kernel microbenchmarks still correctly record
zero project dependencies: their sources include only standard/system headers
and do not consume public solver types. Thus zero dependencies is diagnostic
only for objects known to include repository headers. All twelve candidates
were rebuilt; the four standalone harnesses were not evidence of the ABI fault.


The imported Unicode-path CI smoke initially referenced untracked
`tests/data/miplib2017/benchmark/nw04.mps.gz`, absent from a clean checkout and
this machine. This checks process/path handling, not MIPLIB accuracy. An initial trial with tracked NETLIB `afiro.mps` confirmed the JSON encoding
failure but was unsuitable for the MILP harness because it has no integer
variables. Use a dedicated, tracked one-variable MIP fixture instead.


## Unicode serialization rationale (fixed before the portability edit)

Model: JSON strings are UTF-8 (RFC 8259, Section 8.1); Windows native filesystem
paths are UTF-16. C++ `filesystem::path::u8string` supplies the UTF-8 boundary,
whereas `path::string` uses the local Windows encoding. The trial failed with
JSON error 316 on a Chinese directory name before producing usable evidence.
Convert path metadata explicitly to UTF-8. Cost: O(path length) conversion and
storage only; zero additional optimization work and no timing gain predicted.

Fixture: minimize x subject to x >= 0.5, 0 <= x <= 1 and x integer. Its unique
feasible integer point and optimum are x = 1, so the predicted objective is 1.
Use `tests/data/windows_worker_smoke.mps` under a Chinese directory; require
exit 0, exact recovered directory, one audited optimal result, objective within
1e-6 of 1, hard-deadline supervision and no concurrent tree by default. Rebuild
only `miplib2017_benchmark`; the passed 22-target suite is unaffected by this
isolated serialization edit.


## Measured versus predicted

Platform: Windows x64; MSVC 19.44.35207; CMake Ninja Multi-Config, Release;
`/O2 /Ob2 /fp:fast /MD`, Eigen maximum alignment 32;
MKL ON (INTEL threading), IPO OFF, native architecture OFF, PaPILO ON,
local Ipopt ON (`pardisomkl`), tests ON; CPLEX/Gurobi/Python/SCUC options OFF.
Source is the merge of `253d82b0` and `8fdea5a1` recorded by the commit containing
this report. A post-commit machine-readable record with its exact commit ID is
saved locally as `reports/main-sync-20260917-evidence.json`.

| Metric | Prediction | Measurement |
|---|---|---|
| Release build | success | success after invalidating stale objects |
| Registered CTest targets | all pass | 22/22, 369.77 seconds wall time |
| Decomposition oracle tests | existing 1e-6 checks pass | `test_benders_decomposition` passed |
| R3 evidence validator | all pass | 19/19 |
| Documentation anchors | zero failures | 0 (787 file-symbol, 187 path anchors) |
| Unicode worker objective | 1 within 1e-6 | 1; optimal and original-model audit passed |
| Unicode directory round trip | identical | identical |
| Hard-deadline wrapper | timeout exit 124 | exit 124 |
| Added-line completion marker guard | zero matches | 0 |

Commands:

```powershell
cmake --build build/windows-msvc-release --config Release --parallel 4
ctest --test-dir build/windows-msvc-release -C Release --output-on-failure -j 2
python -m unittest discover -s benchmark -p test_r3_gate.py -v
python tools/doc_anchor_check.py
python tools/run_with_hard_deadline.py --timeout 0.1 -- python -c "import time; time.sleep(10)"
cmake --build build/windows-msvc-release --config Release --parallel 4 --target miplib2017_benchmark
$unicodeDir = Join-Path (Get-Location).Path 'reports/main-sync-20260917-路径'
New-Item -ItemType Directory -Force -Path $unicodeDir | Out-Null
Copy-Item tests/data/windows_worker_smoke.mps $unicodeDir
tests/Release/miplib2017_benchmark.exe --data-dir $unicodeDir --solvers native-highs-lp --case windows_worker_smoke --repeat 1 --time-limit 1 --hard-timeout-grace 2 --max-nodes 20 --json (Join-Path $unicodeDir 'result.json')
```

Logs: `reports/main-sync-20260917-build*.log`, `*-ctest*.log`, `*-r3.log`,
`*-unicode*.log`; the Unicode result is in the command's output directory.
These are integration/correctness results, not a stable-hardware performance
comparison or R3 performance approval. Python bindings were imported and the
Python driver scripts passed syntax checks; Python runtime execution and
CPLEX-enabled runtime execution were not exercised in this configuration.
The SDK and downstream applications must rebuild against the new public
headers; no downstream simulation or installed SDK replacement was performed.

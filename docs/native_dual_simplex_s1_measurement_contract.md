# Native Dual Simplex — S1 Measurement Contract (2026-08-04)

Companion to `native_dual_simplex_system_roadmap_2026-08-04.md` (§5, §13, §15).
S0 (bound-domain correctness) is merged; this document fixes the reproducible
measurement schema, trace and baseline that every S2+ change is measured
against. No pivot-architecture change is promoted without a report that
conforms to this contract.

## 1. Fixed measurement conditions

The serial comparison is authoritatively **one-thread Native vs one-thread
HiGHS**, same binary, same presolve, same resource budget.

| Condition | Value | Where enforced |
|---|---|---|
| Build type | `Release` (`-O3 -DNDEBUG`) | CMakeCache; recorded in report `provenance.build_type` |
| Compiler | clang 21.0.0 (`/usr/bin/c++`) | recorded in `provenance.compiler` |
| Host / arch | Apple M4, Darwin 25.5.0, arm64 | `provenance.target_arch`; host noted in run log |
| Threads | 1 (HiGHS `threads=1`; Native serial) | `configuration.single_threaded_highs=true` |
| Time limit | 30 s per solve | `--time-limit 30`; `configuration.time_limit_sec` |
| Repeats | 3 (A/B/A fleet gate) | `--repeat 3`; `configuration.repeats` |
| Presolve | HiGHS presolve for both; Native consumes the presolved standard form | `Native-DualSimplex(+HiGHS-presolve)` runner |
| Accuracy gate | success ∧ `rel.obj ≤ 1e-5` ∧ normalized primal violation ≤ `1e-7` | `configuration.*_tolerance` |
| Fleet | 24 NETLIB cases in `tests/data/netlib` + `mps_manifest.csv` references | `load_cases` |

Provenance (git commit, build, compiler, arch, timestamp, command, schema
version) is embedded in **every** JSON report (`root.provenance`), so a number
is always tied to the exact binary and invocation that produced it. Because a
git subprocess is not always reachable (sandbox/CI), pass the commit explicitly:

```sh
MIPSOLVERS_BENCH_GIT_COMMIT="$(git rev-parse HEAD)" ./tests/netlib_solver_benchmark ...
```

## 2. Reproduction commands

Fleet baseline (Native vs HiGHS, 24×3):

```sh
MIPSOLVERS_BENCH_GIT_COMMIT="$(git rev-parse HEAD)" \
  ./tests/netlib_solver_benchmark --repeat 3 --time-limit 30 \
    --solvers native-dual-simplex,highs-simplex \
    --csv  reports/s1_baseline_<commit>_repeat3.csv \
    --json reports/s1_baseline_<commit>_repeat3.json
```

Per-phase critical-path profile for the representative cases (stderr `[DS-*]`):

```sh
for c in d2q06c degen3 25fv47; do
  MIPSOLVERS_DS_PROFILE=1 ./tests/netlib_solver_benchmark \
    --case "$c" --repeat 1 --time-limit 30 --solvers native-dual-simplex
done
```

The path-contract check compares two report JSONs with all timing fields
stripped: a Class P change must leave every non-time field byte-identical.

## 3. Result-log schema (roadmap §15 → captured fields)

| §15 field | Source | Status |
|---|---|---|
| binary / config hash | `provenance.git_commit` + `provenance` block | captured |
| aggregation-script version | `provenance.schema_version` (`s1-measurement-contract-1`) | captured |
| Correctness (tests / NETLIB / SCUC / metamorphic) | `test_dual_simplex`, `test_netlib_regression`, `test_scuc_module`, `[bound_domain][metamorphic]` | captured |
| Trace: pivot / rebuild / phase / BFRT / termination | `[DS-PROFILE]`, `[DS-PHASES]`, `[DS-BFRT]`, `[DS-PHASE-I-TERMINAL]` | captured |
| Setup / pivots / per-pivot / rebuild / cleanup / cert | `Statistics` + `[DS-PROFILE]` buckets + `native_kernel_ms_per_dual_pivot` | captured |
| Serial A/B/A: Native / HiGHS / ratio | fleet `summary.geometric_mean_ms`, `geometric_speedup_vs_highs_simplex` | captured |
| PRICE / BFRT / DSE / solve / commit time + support | `[DS-PROFILE]` (price, entering/BFRT, dse, ftran, rcUpdate), `[DS-BFRT]`, `[DS-DENSITY]` | captured |
| Reinversion R, u, interval | `[DS-REINVERT-MODEL]` (per phase) | captured |
| Kernel: dynamic instructions / effective bytes | — | **deferred (§5, see gaps)** |
| Profile: memory-stall / non-memory-stall normalized time | — | **deferred (§5, see gaps)** |
| Parallel equal-thread A/B/A | `--threads` cohort (S4) | future stage |
| Warm cohorts (root / node / strong-branch) | S5 harness | future stage |

`T = T_setup + K·C_critical-path + Σ R_r + T_cleanup + T_cert` (roadmap §1) is
observable today except for the two deferred kernel terms (dynamic
instructions, memory-stall separation).

## 4. Fixed baseline (post-S0, git `a7ee4208`)

Fleet, 24×3, one-thread, 30 s. Reports:
`reports/s1_baseline_a7ee420_repeat3.{csv,json}`,
`reports/s1_baseline_phase_profile_a7ee420.txt`.

| Solver | Median (ms) | GeoMean (ms) | vs HiGHS | Accurate |
|---|---:|---:|---:|---:|
| HiGHS-simplex | 3.17 | 3.57–3.64 | 1.000x | 72/72 |
| Native-DualSimplex | 3.79–3.88 | 4.38–4.46 | **0.815–0.823x** | 72/72 |

Consistent with the roadmap §2 reference (0.819x, 72/72). S0 left every
representative case's `dual_pivots` unchanged (regression-clean / Class P on the
NETLIB fleet). Representative critical path (post-S0):

| Case | m×n | wall | pivots | dualI / dualII / cleanup | minor buckets (price / BFRT / dse / ftran / leaving) |
|---|---|---:|---:|---|---|
| `d2q06c` | 1759×6423 | 0.86 s | 5609 | 1423 / 4186 / 9 | 0.21 / 0.16 / 0.10 / 0.08 / 0.07 |
| `degen3` | 1405×3203 | 0.15 s | 2082 | 465 / 1617 / 0 | 0.03 / 0.02 / 0.02 / 0.02 / 0.02 |
| `25fv47` | 673×2133 | 0.18 s | 2181 | 0 / 2181 / 833 (E1, startShifts=136) | 0.03 / 0.02 / 0.02 / 0.01 / 0.01 |

`d2q06c` per-pivot cost 0.153 ms; the largest single minor bucket is
PRICE (`A^T·rEP`, 0.21 s), then BFRT (0.16 s) — matching the roadmap's finding
that the primary gap is per-pivot architecture / critical-path, not pivot count.

## 5. Instrumentation gaps deferred to S2/S3

Per the roadmap decision (portable half now), these §5 requirements are built
when the first independent-kernel experiment (S2/S3) needs the §13.3 structural
gate, on the platform chosen then:

1. **Per-solve dynamic instructions + effective read/write bytes.** Today only
   the BFRT SIMD micro-kernel is instruction/byte-audited (in
   `native_dual_simplex_kernel_speedup_plan.md`, via the reference-vs-NEON
   benchmark). A general per-solve counter needs macOS `Instruments`/`ktrace`
   or a Linux `perf` box; Apple Silicon has no simple `perf stat`.
2. **Memory-stall vs non-memory-stall normalized time.** Required so a change is
   not misattributed to memory traffic when the real cause is frequency,
   branch or cache behaviour. Same tooling constraint as (1).

Until then, an independent kernel is gated on wall time + the path contract +
the analytic instruction/byte model recorded in the kernel-speedup doc; a
production-promotion still requires the §13.3 dual (instructions ∧ bytes) drop
once the counter tooling exists.

## 6. Promotion protocol (roadmap §13)

- Correctness gate: full native tests, NETLIB 72/72, SCUC audit, sentinel
  metamorphic tests, certificate/ray + cold/warm + rebuild/fallback coverage.
- Class P: identical pivot / rebuild / phase / BFRT flip-shift / termination
  trace (byte-identical timing-stripped report).
- Class A: stated mathematical contract, allowed trace changes, re-recorded
  baseline, out-of-sample gate.
- Conclusion is one of `retain` / `diagnostic-only` / `reject` /
  `blocked-by-prerequisite`, and must name which cost-model term it moves.

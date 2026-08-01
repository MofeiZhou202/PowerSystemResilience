# NETLIB cross-solver benchmark (2026-07-31)

## Scope and method

This benchmark covers all 24 continuous LP instances under `tests/data/netlib`
using the Release build on macOS arm64 (Apple clang 21). The linked backends
were HiGHS 1.14.0, SCIP 9.0.0, and Ipopt 3.14.20. Gurobi was disabled in this
build.

An answer is `accurate` only if the backend reports success, relative objective
error is at most `1e-5`, and independently audited primal violation divided by
`max(1, |rhs|)` is at most `1e-7`. This is intentionally stricter than merely
trusting the backend status. HiGHS direct runs use one thread. Native dual
simplex and Native IPM are labeled `+HiGHS-presolve` because they use the
project's production-like presolve option; their numerical kernels remain
native.

The full algorithm matrix used one run per case, a 30 second supported-backend
limit, and 100,000 iterations for first-order/native simplex methods (Ipopt and
LCQP are capped at 2,000 iterations). The five core paths were then repeated
three times after passing the same in-memory `LPModel` to HiGHS and Native.
SCIP-direct includes reading the original MPS and is primarily an adapter
diagnostic rather than a parser-free kernel timing.

Raw artifacts:

- `reports/netlib_benchmark.csv` and `.json`: 24 x 9 full matrix (216 runs)
- `reports/netlib_core_repeat3.csv` and `.json`: five core paths, three repeats
  (360 runs)
- `reports/netlib_scip_direct.csv` and `.json`: SCIP original-MPS control

## Correctness and robustness

| Algorithm/path | Backend success | Strictly accurate | Main failures |
|---|---:|---:|---|
| HiGHS simplex | 24/24 | 24/24 | None |
| HiGHS IPM (no crossover) | 24/24 | 24/24 | None |
| SCIP direct original MPS | 24/24 | 24/24 | None |
| Native dual simplex + presolve | 22/24 | 22/24 | `d2q06c`, `grow22` |
| Native IPM + presolve | 22/24 | 22/24 | `agg`, `lotfi` numerical errors |
| Ipopt, LP represented as NLP | 20/24 | 17/24 | 4 iteration limits; 3 loose-feasibility results |
| HiGHS PDLP | 22/24 | 16/24 | `d2q06c`, `pilot4` time limits; 6 residual misses |
| Native LCQP on LP | 14/24 | 14/24 | 10 residual-divergence failures |
| SCIP through current adapter | 23/24 | 14/24 | 9 invalid returned solutions; `pilot4` fails |
| Native PDLP | 0/24 | 0/24 | All cases reach 100,000-iteration limit |

The SCIP split is decisive. SCIP itself solves the original 24 MPS files
accurately, but `ScipAdapter::solve_milp()` rewrites the in-memory LP to a
temporary MPS and only 14 results pass the independent audit. Models such as
`adlittle`, `agg`, `fit1d`, and `kb2` return a nominal `Solved` status with
large original-model violations. The temporary MPS writer/ranged-row or free-
variable path needs correction; this is not evidence of a SCIP optimizer
failure.

## Core timing

Three-repeat aggregate timing is shown below. Geometric speedup is relative to
HiGHS simplex on cases where both compared paths are accurate; therefore the
Native figures exclude their two failed cases and must not be read as a
reliability-adjusted portfolio score.

| Core path | Accurate runs | Median (ms) | Geomean (ms) | Speedup vs HiGHS simplex |
|---|---:|---:|---:|---:|
| HiGHS simplex | 72/72 | 2.971 | 3.367 | 1.000x |
| HiGHS IPM | 72/72 | 4.911 | 4.999 | 0.674x |
| Native IPM + presolve | 66/72 | 4.176 | 4.100 | 1.081x on common accurate cases |
| Native dual simplex + presolve | 66/72 | 8.028 | 7.680 | 0.447x on common accurate cases |
| SCIP current adapter | 42/72 | 12.046 | 15.152 | 0.234x on common accurate cases |

Across per-case medians, Native IPM is the fastest accurate path on 11 cases,
Native dual simplex on 4, HiGHS simplex on 7, and HiGHS IPM on 2. Reliability
changes the operational conclusion: only the two HiGHS paths are accurate on
all 24 cases. SCIP-direct has an 11.040 ms median and 15.132 ms geomean while
including original-MPS parsing; it is about 4.58x slower than in-memory HiGHS
simplex geometrically on this small/medium suite.

Large-instance medians illustrate the scaling differences:

| Case | HiGHS simplex | HiGHS IPM | Native dual | Native IPM |
|---|---:|---:|---:|---:|
| `25fv47` | 65.9 ms | 47.0 ms | 714.0 ms | 45.1 ms |
| `d2q06c` | 303.1 ms | 174.6 ms | failed | 190.8 ms |
| `degen3` | 68.5 ms | 65.4 ms | 2284.9 ms | 108.7 ms |
| `pilot4` | 14.9 ms | 20.3 ms | 133.8 ms | 21.8 ms |

Ipopt demonstrates why a general NLP solver should not be the automatic LP
route. It obtains 17 strict answers and can solve `d2q06c`, but that case takes
22.6 seconds versus 0.175 seconds for HiGHS IPM. Native LCQP also solves
`d2q06c` accurately but takes 35.5 seconds. The sub-millisecond LCQP median is
misleading because ten harder cases fail quickly.

## Recommendations

1. Keep HiGHS simplex as the reliability baseline and production fallback.
2. Consider Native IPM as an opportunistic fast path only with the existing
   residual audit and automatic HiGHS fallback; fix the deterministic `agg`
   and `lotfi` failures before broadening its default routing.
3. Fix the SCIP adapter's LP-to-MPS export and add ranged/free-variable
   regressions using the ten discrepant instances. Direct SCIP is healthy.
4. Treat Native dual simplex as experimental until `d2q06c` Phase I and
   `grow22` reconstruction are fixed; its large degenerate-case performance is
   also substantially behind HiGHS.
5. Do not route generic LPs to Native PDLP, LCQP, or Ipopt by default. HiGHS
   PDLP is useful only for selected very large sparse models with a looser
   feasibility target and a factorization-based fallback.

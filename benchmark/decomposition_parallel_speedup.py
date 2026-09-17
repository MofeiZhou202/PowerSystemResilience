#!/usr/bin/env python3
"""Parallel speedup benchmark for two-stage Benders decomposition.

Builds a facility-location instance with a transportation recourse (a genuine
LP per scenario, unlike the trivial newsvendor tests) and times multi-cut
Benders as the ``threads`` option is increased. The objective must be identical
across thread counts (parallelism only reorders independent subproblem solves).

    python benchmark/decomposition_parallel_speedup.py [n_fac] [n_dem] [n_scen]

Defaults: 10 facilities, 12 demand nodes, 60 scenarios.
"""
from __future__ import annotations

import glob
import os
import sys
import time

import numpy as np


def _locate_extension() -> None:
    try:
        import mipsolvers  # noqa: F401

        return
    except ImportError:
        pass
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for pat in ("build*/mipsolvers*.so", "build*/**/mipsolvers*.so"):
        for so in glob.glob(os.path.join(root, pat), recursive=True):
            sys.path.insert(0, os.path.dirname(so))
            return


_locate_extension()
import mipsolvers  # noqa: E402

dec = mipsolvers.decomposition


def build_instance(n_fac: int, n_dem: int, n_scen: int, seed: int = 7):
    rng = np.random.default_rng(seed)
    open_cost = rng.uniform(5.0, 15.0, size=n_fac)
    cap = rng.uniform(8.0, 16.0, size=n_fac)
    flow_cost = rng.uniform(1.0, 4.0, size=(n_fac, n_dem))
    penalty = 50.0

    n = n_fac * n_dem + n_dem  # flows + shortfalls
    m = n_fac + n_dem          # supply rows + demand rows

    def ff(i, j):
        return i * n_dem + j

    def sh(j):
        return n_fac * n_dem + j

    # Recourse cost d and structural matrices W, T (shared across scenarios).
    d = np.zeros(n)
    for i in range(n_fac):
        for j in range(n_dem):
            d[ff(i, j)] = flow_cost[i, j]
    for j in range(n_dem):
        d[sh(j)] = penalty

    W = np.zeros((m, n))
    T = np.zeros((m, n_fac))
    for i in range(n_fac):                       # supply: -sum_j f_ij >= -cap_i x_i
        for j in range(n_dem):
            W[i, ff(i, j)] = -1.0
        T[i, i] = cap[i]
    for j in range(n_dem):                       # demand: sum_i f_ij + short_j >= d_j
        for i in range(n_fac):
            W[n_fac + j, ff(i, j)] = 1.0
        W[n_fac + j, sh(j)] = 1.0

    lb = np.zeros(n)
    ub = np.full(n, 1e6)
    vtype = "C" * n

    first = dict(c=open_cost, vtype="B" * n_fac)  # binary open decisions
    scenarios = []
    for _ in range(n_scen):
        demand = rng.uniform(2.0, 10.0, size=n_dem)
        h = np.zeros(m)
        h[n_fac:] = demand
        scenarios.append(dict(d=d, T=T, W=W, h=h, lb=lb, ub=ub, vtype=vtype,
                              prob=1.0 / n_scen))
    return first, scenarios


def main() -> int:
    n_fac = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    n_dem = int(sys.argv[2]) if len(sys.argv) > 2 else 12
    n_scen = int(sys.argv[3]) if len(sys.argv) > 3 else 60
    first, scenarios = build_instance(n_fac, n_dem, n_scen)
    print(f"facility-location + transportation recourse: "
          f"{n_fac} facilities, {n_dem} demands, {n_scen} scenarios "
          f"(recourse LP: {n_fac * n_dem + n_dem} vars, {n_fac + n_dem} rows)")

    baseline_obj = None
    t1 = None
    print(f"{'threads':>8} {'time_s':>10} {'speedup':>9} {'objective':>14} {'iters':>6}")
    for threads in (1, 2, 4, 8):
        best = min(
            (_run(first, scenarios, threads) for _ in range(3)),
            key=lambda r: r[0],
        )
        elapsed, res = best
        if t1 is None:
            t1 = elapsed
        if baseline_obj is None:
            baseline_obj = res["objective"]
        elif abs(res["objective"] - baseline_obj) > 1e-6 * max(1.0, abs(baseline_obj)):
            print(f"  OBJECTIVE MISMATCH at threads={threads}: "
                  f"{res['objective']} vs {baseline_obj}")
            return 1
        print(f"{threads:>8} {elapsed:>10.4f} {t1 / elapsed:>8.2f}x "
              f"{res['objective']:>14.4f} {res['iterations']:>6}")
    print("RESULT: PASS (objective invariant across thread counts)")
    return 0


def _run(first, scenarios, threads):
    t0 = time.perf_counter()
    res = dec.solve_stochastic(first, scenarios, method="benders",
                               cut_mode="multi", threads=threads)
    return time.perf_counter() - t0, res


if __name__ == "__main__":
    raise SystemExit(main())

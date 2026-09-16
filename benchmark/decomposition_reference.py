#!/usr/bin/env python3
"""Reference regression for the two-stage decomposition module.

Encodes a small two-stage instance in the same variable convention as the
MATLAB prototypes in ``DecompositionAlgorithms/`` (first stage ``c``; recourse
``T``, ``W``, ``h``, ``d``; scenario probabilities ``prob``; coupling
``W y >= h - T x``) and checks that every solver agrees:

  * stochastic:  extensive form == Benders multi-cut == Benders single-cut
  * robust:      extensive epigraph == column-and-constraint generation

Run (after building the ``mipsolvers`` extension with -DMIPSOLVERS_BUILD_PYTHON=ON):

    python benchmark/decomposition_reference.py

Exits non-zero if any method disagrees, so it doubles as a CI regression guard.
"""
from __future__ import annotations

import glob
import os
import sys

import numpy as np


def _locate_extension() -> None:
    """Add the built mipsolvers extension directory to sys.path."""
    try:
        import mipsolvers  # noqa: F401

        return
    except ImportError:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)
    for pattern in ("build*/mipsolvers*.so", "build*/**/mipsolvers*.so"):
        for so in glob.glob(os.path.join(root, pattern), recursive=True):
            sys.path.insert(0, os.path.dirname(so))
            return


_locate_extension()
import mipsolvers  # noqa: E402

dec = mipsolvers.decomposition

# ── Reference instance (MATLAB two_stage_so_* convention) ────────────────────
# 3 candidate facilities: binary open decision x, opening cost c, capacity cap.
OPEN_COST = [5.0, 8.0, 6.0]
CAPACITY = [4.0, 7.0, 5.0]
SHORT_PENALTY = 10.0  # $/unit of unserved demand (recourse cost d)
# Demand scenarios (value, probability).
DEMANDS = [(3.0, 0.25), (8.0, 0.50), (12.0, 0.25)]

FIRST = dict(c=np.array(OPEN_COST), vtype="BBB")


def scenario(demand: float, prob: float) -> dict:
    # Recourse: min 10*y  s.t.  sum_i cap_i x_i + y >= demand,  y >= 0
    #   => W = [1], T = [cap...], h = [demand]
    return dict(
        d=np.array([SHORT_PENALTY]),
        T=np.array([CAPACITY]),
        W=np.array([[1.0]]),
        h=np.array([demand]),
        lb=np.array([0.0]),
        ub=np.array([1e20]),
        vtype="C",
        prob=prob,
    )


SCENARIOS = [scenario(d, p) for d, p in DEMANDS]


def _check(name: str, a: float, b: float, tol: float = 1e-6) -> bool:
    ok = abs(a - b) <= tol * max(1.0, abs(a))
    print(f"  {name:<34} {a:12.6f} vs {b:12.6f}  {'OK' if ok else 'MISMATCH'}")
    return ok


def main() -> int:
    ok = True

    print("Two-stage stochastic (min c'x + E[Q]):")
    ext = dec.solve_stochastic(FIRST, SCENARIOS, method="extensive")
    ben_m = dec.solve_stochastic(FIRST, SCENARIOS, method="benders", cut_mode="multi")
    ben_s = dec.solve_stochastic(FIRST, SCENARIOS, method="benders", cut_mode="single")
    print(f"  extensive objective = {ext['objective']:.6f} ({ext['status']})")
    print(f"  benders multi-cut   = {ben_m['objective']:.6f} "
          f"[{ben_m['iterations']} iters, {ben_m['cuts_added']} cuts]")
    print(f"  benders single-cut  = {ben_s['objective']:.6f} "
          f"[{ben_s['iterations']} iters, {ben_s['cuts_added']} cuts]")
    ok &= _check("extensive == benders(multi)", ext["objective"], ben_m["objective"])
    ok &= _check("extensive == benders(single)", ext["objective"], ben_s["objective"])

    print("\nTwo-stage robust (min c'x + max_s Q_s):")
    rext = dec.solve_robust(FIRST, SCENARIOS, method="extensive")
    ccg = dec.solve_robust(FIRST, SCENARIOS, method="ccg")
    print(f"  extensive objective = {rext['objective']:.6f} ({rext['status']})")
    print(f"  ccg objective       = {ccg['objective']:.6f} "
          f"[{ccg['iterations']} iters, {ccg['scenarios_generated']} scenarios]")
    ok &= _check("robust extensive == ccg", rext["objective"], ccg["objective"])

    print("\nRESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

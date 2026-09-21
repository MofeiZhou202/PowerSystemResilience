#!/usr/bin/env python3.8
"""bench_01_ieee118_solvers.py — Solver performance comparison on SCUC cases.

Compares HiGHS, SCIP, Gurobi, and NativeBranchAndCut on:
  - IEEE 39-bus  × 24 periods
  - IEEE 118-bus × 24 periods
  - IEEE 118-bus × 96 periods  (4-day horizon)

Usage:
  python3.8 bench_01_ieee118_solvers.py [--solvers S1,S2,...] [--no-gurobi] [--quick]
"""

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

# ── locate mipsolvers module ────────────────────────────────────────────────
ROOT = Path(__file__).resolve().parent.parent  # MIPSolvers/
BUILD_PY = ROOT / "build_py"
CASE_BUILDER = ROOT / "build_mipsolvers" / "scuc_case_builder"

sys.path.insert(0, str(BUILD_PY))
import mipsolvers  # noqa: E402

# ── CLI arguments ───────────────────────────────────────────────────────────
parser = argparse.ArgumentParser(description="SCUC solver benchmark")
parser.add_argument("--no-gurobi", action="store_true",
                    help="Skip Gurobi (if no licence)")
parser.add_argument("--quick", action="store_true",
                    help="Only run ieee39×24 and ieee118×24 (skip 96-period)")
parser.add_argument("--solvers",
                    default="HiGHS,SCIP,NativeBranchAndCut",
                    help="Comma-separated list of solvers to test")
parser.add_argument("--mip-gap", type=float, default=0.005,
                    help="MIP gap tolerance (default 0.5%%)")
parser.add_argument("--time-limit", type=float, default=300.0,
                    help="Time limit per solve (seconds, default 300)")
args = parser.parse_args()

SOLVERS = [s.strip() for s in args.solvers.split(",")]
if not args.no_gurobi and "Gurobi" not in SOLVERS:
    # Try Gurobi by default unless --no-gurobi
    SOLVERS_ALL = SOLVERS + ["Gurobi"]
else:
    SOLVERS_ALL = SOLVERS
if args.no_gurobi:
    SOLVERS_ALL = [s for s in SOLVERS_ALL if s != "Gurobi"]


def build_case_json(case: str, T: int, wind: bool = False) -> str:
    """Generate a SCUC case JSON string via the C++ case_builder tool."""
    cmd = [str(CASE_BUILDER), "--case", case, "--T", str(T), "--compact"]
    if wind:
        cmd.append("--wind")
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    if result.returncode != 0:
        raise RuntimeError(f"case_builder failed: {result.stderr.strip()}")
    return result.stdout.strip()


def run_solver(json_str: str, solver: str, mip_gap: float, time_limit: float):
    """Solve a SCUC instance with a given solver. Returns dict of metrics."""
    data = json.loads(json_str)
    data["config"]["solver"]        = solver
    data["config"]["mip_gap"]       = mip_gap
    data["config"]["time_limit_sec"] = time_limit
    modified_json = json.dumps(data)

    t0 = time.perf_counter()
    try:
        out_json = mipsolvers.scuc.solve_json(modified_json)
        elapsed = time.perf_counter() - t0
        out = json.loads(out_json)
        scuc = out.get("scuc", {})
        return {
            "solver":    solver,
            "converged": scuc.get("converged", False),
            "cost":      scuc.get("objective", float("nan")),
            "mip_gap":   scuc.get("mip_gap", float("nan")),
            "time_s":    elapsed,
            "error":     None,
        }
    except Exception as e:
        elapsed = time.perf_counter() - t0
        return {
            "solver":    solver,
            "converged": False,
            "cost":      float("nan"),
            "mip_gap":   float("nan"),
            "time_s":    elapsed,
            "error":     str(e)[:80],
        }


def print_table(rows: list, case_label: str):
    hdr = f"\n{'='*72}\n  {case_label}\n{'='*72}"
    print(hdr)
    print(f"  {'Solver':<22} {'Converged':<10} {'Cost ($)':<16} "
          f"{'MIP gap':<10} {'Time (s)':<10} Note")
    print(f"  {'-'*22} {'-'*10} {'-'*16} {'-'*10} {'-'*10} ----")
    ref_time = None
    for r in rows:
        conv_str = "YES" if r["converged"] else "NO "
        cost_str = f"{r['cost']:>15,.0f}" if r["cost"] == r["cost"] else "       N/A"
        gap_str  = f"{r['mip_gap']:.4f}" if r["mip_gap"] == r["mip_gap"] else "  N/A"
        note = r["error"] or ""
        if r["converged"] and ref_time is None:
            ref_time = r["time_s"]
        speedup = f"{ref_time / r['time_s']:.2f}x" if ref_time and r["time_s"] > 0 else ""
        print(f"  {r['solver']:<22} {conv_str:<10} {cost_str:<16} "
              f"{gap_str:<10} {r['time_s']:<10.1f} {speedup} {note}")
    print()


# ── Cases to benchmark ───────────────────────────────────────────────────────
cases = [
    ("ieee39",  24, "IEEE  39-bus × 24 periods"),
    # ("ieee118", 24, "IEEE 118-bus × 24 periods"),
]
if not args.quick:
    cases.append(("ieee118", 96, "IEEE 118-bus × 96 periods (4-day)"))

print("\n" + "=" * 72)
print("  SCUC Solver Benchmark")
print(f"  Solvers: {', '.join(SOLVERS_ALL)}")
print(f"  MIP gap target: {args.mip_gap:.3%}  |  Time limit: {args.time_limit:.0f}s")
print("=" * 72)

for case_name, T, label in cases:
    print(f"\n[Building] {label} ...", end=" ", flush=True)
    t_build = time.perf_counter()
    try:
        json_str = build_case_json(case_name, T)
    except Exception as e:
        print(f"FAILED: {e}")
        continue
    t_build = time.perf_counter() - t_build
    ng = len(json.loads(json_str)["generators"])
    nb = len(json.loads(json_str)["branches"])
    print(f"done ({t_build:.2f}s)  [{ng} gen, {nb} br, T={T}]")

    rows = []
    for solver in SOLVERS_ALL:
        print(f"  Solving with {solver} ...", end=" ", flush=True)
        r = run_solver(json_str, solver, args.mip_gap, args.time_limit)
        conv = "OK" if r["converged"] else "fail"
        print(f"{conv}  ({r['time_s']:.1f}s)")
        rows.append(r)

    print_table(rows, label)

print("Done.\n")
